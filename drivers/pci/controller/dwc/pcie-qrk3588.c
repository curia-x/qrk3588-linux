// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * Quest QRK3588 DesignWare PCIe host controller glue.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "pcie-designware.h"

#define QRK3588_PMU_BIU_IDLE_SFTCON1	0x8110
#define QRK3588_PMU_BIU_IDLE_ACK_STS0	0x8118
#define QRK3588_PMU_BIU_IDLE_STS0	0x8120
#define QRK3588_PMU_PWR_GATE_SFTCON1	0x8150
#define QRK3588_PMU_PWR_GATE_STS0	0x8180
#define QRK3588_PMU_PHP_SFTCON_BIT	BIT(5)
#define QRK3588_PMU_PHP_STATUS_BIT	BIT(21)

#define QRK3588_PCIE_CLIENT_CON		0x0000
#define QRK3588_PCIE_INTR_MASK_LEGACY	0x001c
#define QRK3588_PCIE_RX_INTX_MASK		GENMASK(3, 0)
#define QRK3588_PCIE_HOT_RESET_CTRL	0x0180
#define QRK3588_PCIE_LTSSM_STATUS	0x0300
#define QRK3588_PCIE_LTSSM_STATE		GENMASK(5, 0)
#define QRK3588_PCIE_DEVICE_TYPE		GENMASK(7, 4)
#define QRK3588_PCIE_ROOT_PORT		FIELD_PREP(QRK3588_PCIE_DEVICE_TYPE, 4)
#define QRK3588_PCIE_LTSSM_ENABLE	BIT(2)
#define QRK3588_PCIE_LTSSM_ENHANCE	BIT(4)
#define QRK3588_PCIE_LINK_UP		(BIT(17) | BIT(16))

#define QRK3588_POWER_TIMEOUT_US		100000

struct qrk3588_pcie {
	struct dw_pcie pci;
	void __iomem *apb;
	struct phy *phy;
	struct regmap *pmu;
	struct gpio_desc *perst;
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control_bulk_data local_resets[2];
};

static inline struct qrk3588_pcie *to_qrk3588_pcie(struct dw_pcie *pci)
{
	return container_of(pci, struct qrk3588_pcie, pci);
}

static void qrk3588_hiword_update(void __iomem *base, u32 offset,
				  u32 mask, u32 value)
{
	writel((mask << 16) | (value & mask), base + offset);
}

static int qrk3588_regmap_hiword_update(struct regmap *regmap, u32 offset,
					u32 mask, u32 value)
{
	return regmap_write(regmap, offset, (mask << 16) | (value & mask));
}

static int qrk3588_pcie_power_on(struct qrk3588_pcie *pcie)
{
	u32 value;
	int ret;

	ret = qrk3588_regmap_hiword_update(pcie->pmu,
					   QRK3588_PMU_PWR_GATE_SFTCON1,
					   QRK3588_PMU_PHP_SFTCON_BIT, 0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(pcie->pmu,
				       QRK3588_PMU_PWR_GATE_STS0, value,
				       !(value & QRK3588_PMU_PHP_STATUS_BIT),
				       10, QRK3588_POWER_TIMEOUT_US);
	if (ret)
		return dev_err_probe(pcie->pci.dev, ret,
				     "PD_PHP power-up timed out (status %#x)\n",
				     value);

	ret = qrk3588_regmap_hiword_update(pcie->pmu,
					   QRK3588_PMU_BIU_IDLE_SFTCON1,
					   QRK3588_PMU_PHP_SFTCON_BIT, 0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(pcie->pmu,
				       QRK3588_PMU_BIU_IDLE_ACK_STS0, value,
				       !(value & QRK3588_PMU_PHP_STATUS_BIT),
				       10, QRK3588_POWER_TIMEOUT_US);
	if (ret)
		return dev_err_probe(pcie->pci.dev, ret,
				     "BIU_PHP resume ACK timed out (status %#x)\n",
				     value);

	ret = regmap_read_poll_timeout(pcie->pmu, QRK3588_PMU_BIU_IDLE_STS0,
				       value,
				       !(value & QRK3588_PMU_PHP_STATUS_BIT),
				       10, QRK3588_POWER_TIMEOUT_US);
	if (ret)
		return dev_err_probe(pcie->pci.dev, ret,
				     "BIU_PHP resume timed out (status %#x)\n",
				     value);

	return 0;
}

static bool qrk3588_pcie_link_up(struct dw_pcie *pci)
{
	struct qrk3588_pcie *pcie = to_qrk3588_pcie(pci);
	u32 status = readl(pcie->apb + QRK3588_PCIE_LTSSM_STATUS);

	return (status & QRK3588_PCIE_LINK_UP) == QRK3588_PCIE_LINK_UP;
}

static enum dw_pcie_ltssm qrk3588_pcie_get_ltssm(struct dw_pcie *pci)
{
	struct qrk3588_pcie *pcie = to_qrk3588_pcie(pci);

	return FIELD_GET(QRK3588_PCIE_LTSSM_STATE,
			 readl(pcie->apb + QRK3588_PCIE_LTSSM_STATUS));
}

static int qrk3588_pcie_start_link(struct dw_pcie *pci)
{
	struct qrk3588_pcie *pcie = to_qrk3588_pcie(pci);

	gpiod_set_value_cansleep(pcie->perst, 0);
	msleep(100);
	qrk3588_hiword_update(pcie->apb, QRK3588_PCIE_CLIENT_CON,
			      QRK3588_PCIE_LTSSM_ENABLE,
			      QRK3588_PCIE_LTSSM_ENABLE);
	return 0;
}

static void qrk3588_pcie_stop_link(struct dw_pcie *pci)
{
	struct qrk3588_pcie *pcie = to_qrk3588_pcie(pci);

	qrk3588_hiword_update(pcie->apb, QRK3588_PCIE_CLIENT_CON,
			      QRK3588_PCIE_LTSSM_ENABLE, 0);
	gpiod_set_value_cansleep(pcie->perst, 1);
}

static const struct dw_pcie_ops qrk3588_pcie_ops = {
	.link_up = qrk3588_pcie_link_up,
	.get_ltssm = qrk3588_pcie_get_ltssm,
	.start_link = qrk3588_pcie_start_link,
	.stop_link = qrk3588_pcie_stop_link,
};

static const struct dw_pcie_host_ops qrk3588_pcie_host_ops = {
};

static int qrk3588_pcie_hw_init(struct qrk3588_pcie *pcie)
{
	int ret;

	ret = clk_bulk_prepare_enable(pcie->num_clks, pcie->clks);
	if (ret)
		return dev_err_probe(pcie->pci.dev, ret,
				     "failed to enable PCIe clocks\n");

	ret = qrk3588_pcie_power_on(pcie);
	if (ret)
		goto err_clocks;

	ret = reset_control_bulk_assert(ARRAY_SIZE(pcie->local_resets),
					pcie->local_resets);
	if (ret)
		goto err_clocks;
	usleep_range(10, 20);

	ret = phy_set_mode(pcie->phy, PHY_MODE_PCIE);
	if (ret)
		goto err_clocks;

	ret = phy_init(pcie->phy);
	if (ret)
		goto err_clocks;

	ret = reset_control_bulk_deassert(ARRAY_SIZE(pcie->local_resets),
					  pcie->local_resets);
	if (ret)
		goto err_phy_exit;

	ret = phy_power_on(pcie->phy);
	if (ret)
		goto err_phy_exit;

	qrk3588_hiword_update(pcie->apb, QRK3588_PCIE_CLIENT_CON,
			      QRK3588_PCIE_DEVICE_TYPE, QRK3588_PCIE_ROOT_PORT);
	qrk3588_hiword_update(pcie->apb, QRK3588_PCIE_HOT_RESET_CTRL,
			      QRK3588_PCIE_LTSSM_ENHANCE,
			      QRK3588_PCIE_LTSSM_ENHANCE);
	qrk3588_hiword_update(pcie->apb, QRK3588_PCIE_INTR_MASK_LEGACY,
			      QRK3588_PCIE_RX_INTX_MASK, 0);
	return 0;

err_phy_exit:
	reset_control_bulk_assert(ARRAY_SIZE(pcie->local_resets),
				  pcie->local_resets);
	phy_exit(pcie->phy);
err_clocks:
	clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
	return ret;
}

static void qrk3588_pcie_hw_deinit(struct qrk3588_pcie *pcie)
{
	qrk3588_hiword_update(pcie->apb, QRK3588_PCIE_INTR_MASK_LEGACY,
			      QRK3588_PCIE_RX_INTX_MASK,
			      QRK3588_PCIE_RX_INTX_MASK);
	qrk3588_pcie_stop_link(&pcie->pci);
	phy_power_off(pcie->phy);
	reset_control_bulk_assert(ARRAY_SIZE(pcie->local_resets),
				  pcie->local_resets);
	phy_exit(pcie->phy);
	clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
}

static int qrk3588_pcie_get_resources(struct platform_device *pdev,
				      struct qrk3588_pcie *pcie)
{
	struct device *dev = &pdev->dev;
	static const char * const reset_names[] = {
		"pipe", "power-up",
	};
	int i;

	pcie->apb = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(pcie->apb))
		return PTR_ERR(pcie->apb);
	pcie->phy = devm_phy_get(dev, "pcie");
	if (IS_ERR(pcie->phy))
		return dev_err_probe(dev, PTR_ERR(pcie->phy),
				     "failed to get PCIe PHY\n");
	pcie->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "quest,pmu");
	if (IS_ERR(pcie->pmu))
		return PTR_ERR(pcie->pmu);

	pcie->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pcie->perst))
		return dev_err_probe(dev, PTR_ERR(pcie->perst),
				     "failed to get endpoint reset GPIO\n");

	pcie->num_clks = devm_clk_bulk_get_all(dev, &pcie->clks);
	if (pcie->num_clks < 0)
		return dev_err_probe(dev, pcie->num_clks,
				     "failed to get clocks\n");

	for (i = 0; i < ARRAY_SIZE(pcie->local_resets); i++) {
		pcie->local_resets[i].id = reset_names[i];
		pcie->local_resets[i].rstc =
			devm_reset_control_get_exclusive(dev, reset_names[i]);
		if (IS_ERR(pcie->local_resets[i].rstc))
			return dev_err_probe(dev,
				PTR_ERR(pcie->local_resets[i].rstc),
				"failed to get %s reset\n", reset_names[i]);
	}

	return 0;
}

static int qrk3588_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qrk3588_pcie *pcie;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->pci.dev = dev;
	pcie->pci.ops = &qrk3588_pcie_ops;
	pcie->pci.pp.ops = &qrk3588_pcie_host_ops;
	platform_set_drvdata(pdev, pcie);

	ret = qrk3588_pcie_get_resources(pdev, pcie);
	if (ret)
		return ret;

	ret = qrk3588_pcie_hw_init(pcie);
	if (ret)
		return ret;

	ret = dw_pcie_host_init(&pcie->pci.pp);
	if (ret) {
		qrk3588_pcie_hw_deinit(pcie);
		return ret;
	}

	return 0;
}

static void qrk3588_pcie_remove(struct platform_device *pdev)
{
	struct qrk3588_pcie *pcie = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&pcie->pci.pp);
	qrk3588_pcie_hw_deinit(pcie);
}

static const struct of_device_id qrk3588_pcie_of_match[] = {
	{ .compatible = "quest,qrk3588-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, qrk3588_pcie_of_match);

static struct platform_driver qrk3588_pcie_driver = {
	.probe = qrk3588_pcie_probe,
	.remove = qrk3588_pcie_remove,
	.driver = {
		.name = "qrk3588-pcie",
		.of_match_table = qrk3588_pcie_of_match,
	},
};
module_platform_driver(qrk3588_pcie_driver);

MODULE_DESCRIPTION("Quest QRK3588 DesignWare PCIe host controller");
MODULE_LICENSE("GPL");
