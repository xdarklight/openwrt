// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Intel Gateway SoCs
 *
 * Copyright (c) 2019 Intel Corporation.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/pci_regs.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "../../pci.h"
#include "pcie-designware.h"

#define PORT_AFR_N_FTS_GEN12_DFT		(SZ_128 - 1)
#define PORT_AFR_N_FTS_GEN3			180
#define PORT_AFR_N_FTS_GEN4			196

/* PCIe Application logic Registers */
#define PCIE_APP_CCR				0x10
#define PCIE_APP_CCR_LTSSM_ENABLE		BIT(0)

/* RC Core Debug Register */
#define PCIE_APP_RC_DR				0x14
#define PCIE_APP_RC_DR_DLL_UP			BIT(0)

/* PHY Link Status Register */
#define PCIE_APP_PHY_SR				0x18
#define PCIE_APP_PHY_SR_PHY_LINK_UP		BIT(0)

#define PCIE_APP_MSG_CR				0x30
#define PCIE_APP_MSG_XMT_PM_TURNOFF		BIT(0)

#define PCIE_APP_PMC				0x44
#define PCIE_APP_PMC_IN_L2			BIT(20)

/* AHB Control Register, fixed bus enumeration exception */
#define PCIE_APP_AHB_CTRL			0x78
#define PCIE_APP_AHB_CTRL_BUS_ERROR_SUPPRESS	BIT(0)

#define PCIE_APP_IRNEN				0xF4
#define PCIE_APP_IRNCR				0xF8
#define PCIE_APP_IRN_AER_REPORT			BIT(0)
#define PCIE_APP_IRN_PME			BIT(2)
#define PCIE_APP_IRN_RX_VDM_MSG			BIT(4)
#define PCIE_APP_IRN_PM_TO_ACK			BIT(9)
#define PCIE_APP_IRN_LINK_AUTO_BW_STAT		BIT(11)
#define PCIE_APP_IRN_BW_MGT			BIT(12)
#define PCIE_APP_IRN_INTA			BIT(13)
#define PCIE_APP_IRN_INTB			BIT(14)
#define PCIE_APP_IRN_INTC			BIT(15)
#define PCIE_APP_IRN_INTD			BIT(16)
#define PCIE_APP_IRN_MSG_LTR			BIT(18)
#define PCIE_APP_IRN_SYS_ERR_RC			BIT(29)
#define PCIE_APP_INTX_OFST			12

#define PCIE_APP_IRNICR				0xFC

#define PCIE_APP_IRN_INT \
	(PCIE_APP_IRN_AER_REPORT | PCIE_APP_IRN_PME | \
	PCIE_APP_IRN_RX_VDM_MSG | PCIE_APP_IRN_SYS_ERR_RC | \
	PCIE_APP_IRN_PM_TO_ACK | PCIE_APP_IRN_MSG_LTR | \
	PCIE_APP_IRN_BW_MGT | PCIE_APP_IRN_LINK_AUTO_BW_STAT | \
	PCIE_APP_IRN_INTA | PCIE_APP_IRN_INTB | \
	PCIE_APP_IRN_INTC | PCIE_APP_IRN_INTD)

#define BUS_IATU_OFFSET			SZ_256M
#define RESET_INTERVAL_MS		100

#define PCI_VENDOR_ID_INFINEON		0x15D1
#define PCI_DEVICE_ID_INFINEON_PCIE	0x0011
#define PCI_VENDOR_ID_LANTIQ		0x1BEF
#define PCI_DEVICE_ID_LANTIQ_PCIE	0x0011

struct intel_pcie_soc {
	unsigned int				pcie_ver;
	const struct dw_pcie_ops		dw_pcie_ops;
	const struct dw_pcie_host_ops		dw_pcie_host_ops;
	const struct regmap_access_table	*app_regmap_rd_table;
	bool					phy_resets_core;
};

struct intel_pcie_port {
	struct dw_pcie		pci;
	struct regmap_config	app_regmap_config;
	struct regmap		*app_regmap;
	struct gpio_desc	*reset_gpio;
	u32			rst_intrvl;
	struct clk		*core_clk;
	struct clk_bulk_data	bulk_clks[2];
	struct reset_control	*core_rst;
	struct phy		*phy;
};

static const struct regmap_range intel_pcie_app_regmap_rd_ranges[] = {
	regmap_reg_range(PCIE_APP_CCR, PCIE_APP_PHY_SR),
	regmap_reg_range(PCIE_APP_MSG_CR, PCIE_APP_MSG_CR),
	regmap_reg_range(PCIE_APP_PMC, PCIE_APP_PMC),
	regmap_reg_range(PCIE_APP_IRNEN, PCIE_APP_IRNICR),
};

static const struct regmap_access_table intel_pcie_app_regmap_rd_table = {
	.yes_ranges = intel_pcie_app_regmap_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(intel_pcie_app_regmap_rd_ranges),
};

static const struct regmap_range lantiq_pcie_app_regmap_rd_ranges[] = {
	regmap_reg_range(PCIE_APP_CCR, PCIE_APP_PHY_SR),
	regmap_reg_range(PCIE_APP_MSG_CR, PCIE_APP_MSG_CR),
	regmap_reg_range(PCIE_APP_AHB_CTRL, PCIE_APP_AHB_CTRL),
	regmap_reg_range(PCIE_APP_IRNEN, PCIE_APP_IRNICR),
};

static const struct regmap_access_table lantiq_pcie_app_regmap_rd_table = {
	.yes_ranges = lantiq_pcie_app_regmap_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(lantiq_pcie_app_regmap_rd_ranges),
};

static int lantiq_pcie_own_config_read32(struct pci_bus *bus,
					 unsigned int devfn, int where,
					 int size, u32 *val)
{
	struct pcie_port *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (PCI_SLOT(devfn) > 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	*val = dw_pcie_read_dbi(pci, where, size);

	return PCIBIOS_SUCCESSFUL;
}

static int lantiq_pcie_own_config_write32(struct pci_bus *bus,
					  unsigned int devfn, int where,
					  int size, u32 val)
{
	struct pcie_port *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (PCI_SLOT(devfn) > 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	dw_pcie_write_dbi(pci, where, size, val);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops lantiq_pcie_own_pci_ops = {
	.read = lantiq_pcie_own_config_read32,
	.write = lantiq_pcie_own_config_write32,
};

static void __iomem *lantiq_pcie_child_config_map_bus(struct pci_bus *bus,
						      unsigned int devfn,
						      int where)
{
	struct pcie_port *pp = bus->sysdata;
	u32 addr;

	/*
	 * According to the vendor kernel this implementation matches the
	 * description from "PCI Express Base Specification v1.1" in
	 * "Table 7-1" on page 341.
	 * We are: type 1, only support 8 buses
	 */
	addr = (bus->number & 0x7) << 20;

	addr |= PCI_SLOT(devfn) << 15;
	addr |= PCI_FUNC(devfn) << 12;
	addr |= where & 0xfff;

	return pp->va_cfg0_base + addr;
}

static struct pci_ops lantiq_pcie_child_pci_ops = {
	.map_bus = lantiq_pcie_child_config_map_bus,
	.read = pci_generic_config_read32,
	.write = pci_generic_config_write32,
};

static void intel_pcie_ltssm_enable(struct intel_pcie_port *lpp)
{
	regmap_update_bits(lpp->app_regmap, PCIE_APP_CCR,
			   PCIE_APP_CCR_LTSSM_ENABLE,
			   PCIE_APP_CCR_LTSSM_ENABLE);
}

static void intel_pcie_ltssm_disable(struct intel_pcie_port *lpp)
{
	regmap_update_bits(lpp->app_regmap, PCIE_APP_CCR,
			   PCIE_APP_CCR_LTSSM_ENABLE, 0);
}

static void intel_pcie_link_setup(struct intel_pcie_port *lpp)
{
	u32 val;
	u8 offset = dw_pcie_find_capability(&lpp->pci, PCI_CAP_ID_EXP);

	val = dw_pcie_readl_dbi(&lpp->pci, offset + PCI_EXP_LNKCTL);

	val &= ~(PCI_EXP_LNKCTL_LD | PCI_EXP_LNKCTL_ASPMC);
	dw_pcie_writel_dbi(&lpp->pci, offset + PCI_EXP_LNKCTL, val);
}

static void intel_pcie_init_n_fts(struct dw_pcie *pci)
{
	switch (pci->link_gen) {
	case 3:
		pci->n_fts[1] = PORT_AFR_N_FTS_GEN3;
		break;
	case 4:
		pci->n_fts[1] = PORT_AFR_N_FTS_GEN4;
		break;
	default:
		pci->n_fts[1] = PORT_AFR_N_FTS_GEN12_DFT;
		break;
	}
	pci->n_fts[0] = PORT_AFR_N_FTS_GEN12_DFT;
}

static int intel_pcie_ep_rst_init(struct intel_pcie_port *lpp)
{
	struct device *dev = lpp->pci.dev;
	int ret;

	lpp->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lpp->reset_gpio)) {
		ret = PTR_ERR(lpp->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request PCIe GPIO: %d\n", ret);
		return ret;
	}

	/* Make initial reset last for 100us */
	usleep_range(100, 200);

	return 0;
}

static void intel_pcie_core_rst_assert(struct intel_pcie_port *lpp)
{
	reset_control_assert(lpp->core_rst);
}

static void intel_pcie_core_rst_deassert(struct intel_pcie_port *lpp)
{
	/*
	 * One micro-second delay to make sure the reset pulse
	 * wide enough so that core reset is clean.
	 */
	udelay(1);
	reset_control_deassert(lpp->core_rst);

	/*
	 * Some SoC core reset also reset PHY, more delay needed
	 * to make sure the reset process is done.
	 */
	usleep_range(1000, 2000);
}

static void intel_pcie_device_rst_assert(struct intel_pcie_port *lpp)
{
	gpiod_set_value_cansleep(lpp->reset_gpio, 1);
}

static void intel_pcie_device_rst_deassert(struct intel_pcie_port *lpp)
{
	msleep(lpp->rst_intrvl);
	gpiod_set_value_cansleep(lpp->reset_gpio, 0);
}

static void intel_pcie_core_irq_disable(struct intel_pcie_port *lpp)
{
	regmap_write(lpp->app_regmap, PCIE_APP_IRNEN, 0);
	regmap_write(lpp->app_regmap, PCIE_APP_IRNCR, PCIE_APP_IRN_INT);
}

static int intel_pcie_get_resources(struct platform_device *pdev,
				    const struct intel_pcie_soc *data)
{
	struct intel_pcie_port *lpp = platform_get_drvdata(pdev);
	struct dw_pcie *pci = &lpp->pci;
	struct device *dev = pci->dev;
	void __iomem *app_base;
	int ret;

	lpp->core_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(lpp->core_clk)) {
		ret = PTR_ERR(lpp->core_clk);
		dev_err_probe(dev, ret, "Failed to get the 'pcie' clock: %d\n",
			      ret);
		return ret;
	}

	lpp->bulk_clks[0].id = "pcie_bus";
	lpp->bulk_clks[1].id = "ahb";
	ret = devm_clk_bulk_get_optional(dev, ARRAY_SIZE(lpp->bulk_clks),
					 lpp->bulk_clks);
	if (ret) {
		dev_err_probe(dev, ret,
			      "Failed to the 'ahb' clock or 'pcie_bus' clocks: %d\n",
			      ret);
		return ret;
	}

	if (!data->phy_resets_core) {
		lpp->core_rst = devm_reset_control_get(dev, NULL);
		if (IS_ERR(lpp->core_rst)) {
			ret = PTR_ERR(lpp->core_rst);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get resets: %d\n",
					ret);
			return ret;
		}
	}

	ret = device_property_read_u32(dev, "reset-assert-ms",
				       &lpp->rst_intrvl);
	if (ret)
		lpp->rst_intrvl = RESET_INTERVAL_MS;

	app_base = devm_platform_ioremap_resource_byname(pdev, "app");
	if (IS_ERR(app_base))
		return PTR_ERR(app_base);

	lpp->app_regmap_config.name = "app",
	lpp->app_regmap_config.reg_bits = 8,
	lpp->app_regmap_config.val_bits = 32,
	lpp->app_regmap_config.reg_stride = 4,
	lpp->app_regmap_config.fast_io = true,
	lpp->app_regmap_config.max_register = PCIE_APP_IRNICR,
	lpp->app_regmap_config.rd_table = data->app_regmap_rd_table;
	lpp->app_regmap = devm_regmap_init_mmio(dev, app_base,
						&lpp->app_regmap_config);
	if (IS_ERR(lpp->app_regmap))
		return PTR_ERR(lpp->app_regmap);

	lpp->phy = devm_phy_get(dev, "pcie");
	if (IS_ERR(lpp->phy)) {
		ret = PTR_ERR(lpp->phy);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Couldn't get pcie-phy: %d\n", ret);
		return ret;
	}

	return 0;
}

static int intel_pcie_wait_l2(struct intel_pcie_port *lpp)
{
	u32 value;
	int ret;
	struct dw_pcie *pci = &lpp->pci;

	if (pci->link_gen < 3)
		return 0;

	/* Send PME_TURN_OFF message */
	regmap_update_bits(lpp->app_regmap, PCIE_APP_MSG_CR,
			   PCIE_APP_MSG_XMT_PM_TURNOFF,
			   PCIE_APP_MSG_XMT_PM_TURNOFF);

	/* Read PMC status and wait for falling into L2 link state */
	ret = regmap_read_poll_timeout(lpp->app_regmap, PCIE_APP_PMC,
				       value, value & PCIE_APP_PMC_IN_L2,
				       20, jiffies_to_usecs(5 * HZ));
	if (ret)
		dev_err(lpp->pci.dev, "PCIe link enter L2 timeout!\n");

	return ret;
}

static void intel_pcie_turn_off(struct intel_pcie_port *lpp)
{
	u32 value;

	if (dw_pcie_link_up(&lpp->pci))
		intel_pcie_wait_l2(lpp);

	/* Put endpoint device in reset state */
	intel_pcie_device_rst_assert(lpp);

	value = dw_pcie_readl_dbi(&lpp->pci, PCI_COMMAND);
	value &= PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(&lpp->pci, PCI_COMMAND, value);
}

static int intel_pcie_host_setup(struct intel_pcie_port *lpp)
{
	int ret;
	struct dw_pcie *pci = &lpp->pci;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(lpp->bulk_clks),
				      lpp->bulk_clks);
	if (ret) {
		dev_err(lpp->pci.dev, "AHB/PCIE_BUS clock enable failed: %d\n",
			ret);
		return ret;
	}

	intel_pcie_core_rst_assert(lpp);
	intel_pcie_device_rst_assert(lpp);

	ret = phy_init(lpp->phy);
	if (ret)
		goto phy_init_err;

	ret = phy_power_on(lpp->phy);
	if (ret)
		goto phy_power_on_err;

	intel_pcie_core_rst_deassert(lpp);

	ret = clk_prepare_enable(lpp->core_clk);
	if (ret) {
		dev_err(lpp->pci.dev, "Core clock enable failed: %d\n", ret);
		goto core_clk_err;
	}

	pci->atu_base = pci->dbi_base + 0xC0000;

	intel_pcie_ltssm_disable(lpp);
	intel_pcie_link_setup(lpp);
	intel_pcie_init_n_fts(pci);
	dw_pcie_setup_rc(&pci->pp);
	dw_pcie_upconfig_setup(pci);

	intel_pcie_device_rst_deassert(lpp);
	intel_pcie_ltssm_enable(lpp);

	ret = dw_pcie_wait_for_link(pci);
	if (ret)
		goto app_init_err;

	/* Enable integrated interrupts */
	regmap_update_bits(lpp->app_regmap, PCIE_APP_IRNEN, PCIE_APP_IRN_INT,
			   PCIE_APP_IRN_INT);

	return 0;

app_init_err:
	clk_disable_unprepare(lpp->core_clk);
core_clk_err:
	intel_pcie_core_rst_assert(lpp);
	phy_power_off(lpp->phy);
phy_power_on_err:
	phy_exit(lpp->phy);
phy_init_err:
	clk_bulk_disable_unprepare(ARRAY_SIZE(lpp->bulk_clks), lpp->bulk_clks);
	return ret;
}

static void __intel_pcie_remove(struct intel_pcie_port *lpp)
{
	intel_pcie_core_irq_disable(lpp);
	intel_pcie_turn_off(lpp);
	clk_disable_unprepare(lpp->core_clk);
	intel_pcie_core_rst_assert(lpp);
	phy_power_off(lpp->phy);
	phy_exit(lpp->phy);
	clk_bulk_disable_unprepare(ARRAY_SIZE(lpp->bulk_clks), lpp->bulk_clks);
}

static int intel_pcie_remove(struct platform_device *pdev)
{
	struct intel_pcie_port *lpp = platform_get_drvdata(pdev);
	struct pcie_port *pp = &lpp->pci.pp;

	dw_pcie_host_deinit(pp);
	__intel_pcie_remove(lpp);

	return 0;
}

static int __maybe_unused intel_pcie_suspend_noirq(struct device *dev)
{
	struct intel_pcie_port *lpp = dev_get_drvdata(dev);
	int ret;

	intel_pcie_core_irq_disable(lpp);
	ret = intel_pcie_wait_l2(lpp);
	if (ret)
		return ret;

	clk_disable_unprepare(lpp->core_clk);
	phy_power_off(lpp->phy);
	phy_exit(lpp->phy);
	clk_bulk_disable_unprepare(ARRAY_SIZE(lpp->bulk_clks), lpp->bulk_clks);
	return ret;
}

static int __maybe_unused intel_pcie_resume_noirq(struct device *dev)
{
	struct intel_pcie_port *lpp = dev_get_drvdata(dev);

	return intel_pcie_host_setup(lpp);
}

static int intel_pcie_rc_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct intel_pcie_port *lpp = dev_get_drvdata(pci->dev);

	return intel_pcie_host_setup(lpp);
}

static int lantiq_pcie_rc_init(struct pcie_port *pp)
{
	pp->bridge->ops = &lantiq_pcie_own_pci_ops;
	pp->bridge->child_ops = &lantiq_pcie_child_pci_ops;

	return intel_pcie_rc_init(pp);
}

static u64 intel_pcie_cpu_addr(struct dw_pcie *pcie, u64 cpu_addr)
{
	return cpu_addr + BUS_IATU_OFFSET;
}

static int lantiq_pcie_link_up(struct dw_pcie *pci)
{
	struct intel_pcie_port *lpp = dev_get_drvdata(pci->dev);
	u32 val;
	int ret;

	ret = regmap_read_poll_timeout(lpp->app_regmap, PCIE_APP_PHY_SR,
				       val, val & PCIE_APP_PHY_SR_PHY_LINK_UP,
				       100, 300000);
	if (ret)
		return 0;

	msleep(100);

	/* Check whether the data link is up */
	regmap_read(lpp->app_regmap, PCIE_APP_RC_DR, &val);
	if (!(val & PCIE_APP_RC_DR_DLL_UP))
		return 0;

	val = dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG1);
	if (val & PCIE_PORT_DEBUG1_LINK_IN_TRAINING ||
	    !(val & PCIE_PORT_DEBUG1_LINK_UP))
		return 0;

	return 1;
}

static u32 lantiq_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
				u32 reg, size_t size)
{
	u32 val = ioread32be(base + ALIGN_DOWN(reg, 4));

	if (!IS_ALIGNED(reg, 4) || size != 4) {
		val >>= (BITS_PER_BYTE * (reg & 0x3));
		val &= (BIT_ULL(BITS_PER_BYTE * size) - 1);
	}

	return val;
}

static void lantiq_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
				  u32 reg, size_t size, u32 val)
{
	u32 tmp;

	if (!IS_ALIGNED(reg, 4) || size != 4) {
		u32 shift = (BITS_PER_BYTE * (reg & 0x3));
		tmp = ioread32be(base + ALIGN_DOWN(reg, 4));
		tmp &= ~((BIT_ULL(BITS_PER_BYTE * size) - 1) << shift);
		tmp |= val << shift;
	} else {
		tmp = val;
	}

	iowrite32be(tmp, base + ALIGN_DOWN(reg, 4));
}

static const struct intel_pcie_soc pcie_data = {
	.pcie_ver		= 0x520A,
	.dw_pcie_ops		= {
		.cpu_addr_fixup = intel_pcie_cpu_addr,
	},
	.dw_pcie_host_ops	= {
		.host_init = intel_pcie_rc_init,
	},
	.app_regmap_rd_table	= &intel_pcie_app_regmap_rd_table,
	.phy_resets_core	= false,
};

static int lantiq_msi_host_init(struct pcie_port *pp)
{
	// HACK: needed as long as we don't have the MSI PIC working
	return 0;
}

static const struct intel_pcie_soc lantiq_pcie_data = {
	.pcie_ver		= 0x0,
	.dw_pcie_ops		= {
		.link_up = lantiq_pcie_link_up,
		.read_dbi = lantiq_pcie_read_dbi,
		.write_dbi = lantiq_pcie_write_dbi,
	},
	.dw_pcie_host_ops	= {
		.host_init = lantiq_pcie_rc_init,
		.msi_host_init = lantiq_msi_host_init,
	},
	.app_regmap_rd_table	= &lantiq_pcie_app_regmap_rd_table,
	.phy_resets_core	= true,
};

static int intel_pcie_probe(struct platform_device *pdev)
{
	const struct intel_pcie_soc *data;
	struct device *dev = &pdev->dev;
	struct intel_pcie_port *lpp;
	struct pcie_port *pp;
	struct dw_pcie *pci;
	int ret;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	lpp = devm_kzalloc(dev, sizeof(*lpp), GFP_KERNEL);
	if (!lpp)
		return -ENOMEM;

	platform_set_drvdata(pdev, lpp);
	pci = &lpp->pci;
	pci->dev = dev;
	pp = &pci->pp;

	ret = intel_pcie_get_resources(pdev, data);
	if (ret)
		return ret;

	ret = intel_pcie_ep_rst_init(lpp);
	if (ret)
		return ret;

	pci->ops = &data->dw_pcie_ops;
	pci->version = data->pcie_ver;
	pp->ops = &data->dw_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Cannot initialize host\n");
		return ret;
	}

	return 0;
}

static void lantiq_pcie_fixup_class(struct pci_dev *dev)
{
	dev->class = (PCI_CLASS_BRIDGE_PCI << 8) | (dev->class & 0xff);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INFINEON, PCI_DEVICE_ID_INFINEON_PCIE,
			lantiq_pcie_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_LANTIQ, PCI_DEVICE_ID_LANTIQ_PCIE,
			lantiq_pcie_fixup_class);

static const struct dev_pm_ops intel_pcie_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(intel_pcie_suspend_noirq,
				      intel_pcie_resume_noirq)
};

static const struct of_device_id of_intel_pcie_match[] = {
	{ .compatible = "intel,lgm-pcie", .data = &pcie_data },
	{ .compatible = "lantiq,xrx200-pcie", .data = &lantiq_pcie_data },
	{ .compatible = "lantiq,xrx300-pcie", .data = &lantiq_pcie_data },
	{}
};

static struct platform_driver intel_pcie_driver = {
	.probe = intel_pcie_probe,
	.remove = intel_pcie_remove,
	.driver = {
		.name = "intel-gw-pcie",
		.of_match_table = of_intel_pcie_match,
		.pm = &intel_pcie_pm_ops,
	},
};
builtin_platform_driver(intel_pcie_driver);
