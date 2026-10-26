// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * Quest QRK3588 Combo PIPE PHY in PCIe mode.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define QRK3588_PHP_PCIESEL_CON		0x0100
#define QRK3588_PCIE1L0_PHY_SEL		BIT(0)

#define QRK3588_PIPE_PHY_CON0		0x0000
#define QRK3588_PIPE_PHY_CON1		0x0004
#define QRK3588_PIPE_PHY_CON2		0x0008
#define QRK3588_PIPE_PHY_CON3		0x000c
#define QRK3588_PIPE_PHY_STATUS1		0x0034
#define QRK3588_PIPE_PHY_MODE		GENMASK(3, 2)
#define QRK3588_PIPE_PHY_RATE		GENMASK(5, 4)
#define QRK3588_PIPE_PHY_RATE_GEN2	FIELD_PREP(QRK3588_PIPE_PHY_RATE, 1)
#define QRK3588_PIPE_PHY_MAC_CLKREQ_N	BIT(8)
#define QRK3588_PIPE_PHY_SEL_CLKREQ_N	BIT(3)
#define QRK3588_PIPE_PHY_CLK_SEL		GENMASK(14, 13)
#define QRK3588_PIPE_PHY_CLK_100M	FIELD_PREP(QRK3588_PIPE_PHY_CLK_SEL, 2)
#define QRK3588_PIPE_PHY_REF_SRC		GENMASK(9, 8)
#define QRK3588_PIPE_PHY_REF_OUTPUT	FIELD_PREP(QRK3588_PIPE_PHY_REF_SRC, 2)
#define QRK3588_PIPE_PHY_SEL_TXELECIDLE	BIT(12)
#define QRK3588_PIPE_PHY_SEL_TXDETECTRX	BIT(11)
#define QRK3588_PIPE_PHY_SEL_POWERDOWN	BIT(4)
#define QRK3588_PIPE_PHY_SEL_RATE	BIT(2)
#define QRK3588_PIPE_PHY_DYNAMIC_OVERRIDES \
	(QRK3588_PIPE_PHY_SEL_TXELECIDLE | \
	 QRK3588_PIPE_PHY_SEL_TXDETECTRX | \
	 QRK3588_PIPE_PHY_SEL_POWERDOWN | QRK3588_PIPE_PHY_SEL_RATE)

#define QRK3588_COMBO_PHY_REG005		0x0014
#define QRK3588_COMBO_PHY_MODE		GENMASK(2, 0)
#define QRK3588_COMBO_PHY_REG031		0x007c
#define QRK3588_COMBO_PHY_CKDRV_REF_SRC	BIT(0)
#define QRK3588_COMBO_PHY_STATUS39	0x009c
#define QRK3588_COMBO_PHY_PLL_LOCK	BIT(0)

#define QRK3588_PHY_TIMEOUT_US		100000
#define QRK3588_PHY_REFCLK_HZ		100000000UL

struct qrk3588_combo_phy {
	struct device *dev;
	void __iomem *base;
	void __iomem *grf;
	struct regmap *php_grf;
	struct clk *refclk;
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control_bulk_data resets[3];
};

static void qrk3588_hiword_update(void __iomem *base, u32 offset,
				  u32 mask, u32 value)
{
	writel((mask << 16) | (value & mask), base + offset);
}

static void qrk3588_combo_phy_configure(struct qrk3588_combo_phy *qphy)
{
	qrk3588_hiword_update(qphy->grf, QRK3588_PIPE_PHY_CON0,
			      QRK3588_PIPE_PHY_MODE | QRK3588_PIPE_PHY_RATE,
			      QRK3588_PIPE_PHY_RATE_GEN2);
	qrk3588_hiword_update(qphy->grf, QRK3588_PIPE_PHY_CON1,
			      QRK3588_PIPE_PHY_CLK_SEL,
			      QRK3588_PIPE_PHY_CLK_100M);

	/* Keep REFCLK present while the endpoint is held in fundamental reset. */
	qrk3588_hiword_update(qphy->grf, QRK3588_PIPE_PHY_CON0,
			      QRK3588_PIPE_PHY_MAC_CLKREQ_N, 0);
	qrk3588_hiword_update(qphy->grf, QRK3588_PIPE_PHY_CON2,
			      QRK3588_PIPE_PHY_SEL_CLKREQ_N,
			      QRK3588_PIPE_PHY_SEL_CLKREQ_N);

	/* Dynamic training controls must be owned by the controller LTSSM. */
	qrk3588_hiword_update(qphy->grf, QRK3588_PIPE_PHY_CON2,
			      QRK3588_PIPE_PHY_DYNAMIC_OVERRIDES, 0);
	qrk3588_hiword_update(qphy->grf, QRK3588_PIPE_PHY_CON3,
			      QRK3588_PIPE_PHY_REF_SRC,
			      QRK3588_PIPE_PHY_REF_OUTPUT);

	/* The PMA protocol mode follows PIPE; an APB override breaks training. */
	writel(readl(qphy->base + QRK3588_COMBO_PHY_REG005) &
	       ~QRK3588_COMBO_PHY_MODE,
	       qphy->base + QRK3588_COMBO_PHY_REG005);
	writel(readl(qphy->base + QRK3588_COMBO_PHY_REG031) |
	       QRK3588_COMBO_PHY_CKDRV_REF_SRC,
	       qphy->base + QRK3588_COMBO_PHY_REG031);
}

static int qrk3588_combo_phy_wait(struct qrk3588_combo_phy *qphy)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(qphy->base + QRK3588_COMBO_PHY_STATUS39,
				 status, status & QRK3588_COMBO_PHY_PLL_LOCK,
				 10, QRK3588_PHY_TIMEOUT_US);
	if (ret)
		dev_err(qphy->dev,
			"Combo PHY PLL timeout: status=%#x con1=%#x con3=%#x\n",
			status, readl(qphy->grf + QRK3588_PIPE_PHY_CON1),
			readl(qphy->grf + QRK3588_PIPE_PHY_CON3));

	return ret;
}

static int qrk3588_combo_phy_init(struct phy *phy)
{
	struct qrk3588_combo_phy *qphy = phy_get_drvdata(phy);
	int ret;

	ret = clk_set_rate(qphy->refclk, QRK3588_PHY_REFCLK_HZ);
	if (ret)
		return ret;
	if (clk_get_rate(qphy->refclk) != QRK3588_PHY_REFCLK_HZ)
		return -ERANGE;

	ret = clk_bulk_prepare_enable(qphy->num_clks, qphy->clks);
	if (ret)
		return ret;

	ret = reset_control_bulk_assert(ARRAY_SIZE(qphy->resets), qphy->resets);
	if (ret)
		goto err_clocks;
	usleep_range(10, 20);
	ret = reset_control_deassert(qphy->resets[0].rstc);
	if (ret)
		goto err_resets;
	ret = reset_control_deassert(qphy->resets[1].rstc);
	if (ret)
		goto err_resets;
	usleep_range(10, 20);

	/* Only the PHY wired through the PHP PCIe mux supplies this syscon. */
	if (qphy->php_grf) {
		ret = regmap_write(qphy->php_grf, QRK3588_PHP_PCIESEL_CON,
				   QRK3588_PCIE1L0_PHY_SEL << 16);
		if (ret)
			goto err_resets;
	}

	qrk3588_combo_phy_configure(qphy);
	ret = reset_control_deassert(qphy->resets[2].rstc);
	if (ret)
		goto err_resets;
	usleep_range(10, 20);

	return 0;

err_resets:
	reset_control_bulk_assert(ARRAY_SIZE(qphy->resets), qphy->resets);
err_clocks:
	clk_bulk_disable_unprepare(qphy->num_clks, qphy->clks);
	return ret;
}

static int qrk3588_combo_phy_power_on(struct phy *phy)
{
	/* The consumer releases its PIPE resets before checking PLL lock. */
	return qrk3588_combo_phy_wait(phy_get_drvdata(phy));
}

static int qrk3588_combo_phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	return mode == PHY_MODE_PCIE && !submode ? 0 : -EINVAL;
}

static int qrk3588_combo_phy_exit(struct phy *phy)
{
	struct qrk3588_combo_phy *qphy = phy_get_drvdata(phy);
	int ret;

	ret = reset_control_bulk_assert(ARRAY_SIZE(qphy->resets), qphy->resets);
	clk_bulk_disable_unprepare(qphy->num_clks, qphy->clks);
	return ret;
}

static const struct phy_ops qrk3588_combo_phy_ops = {
	.init = qrk3588_combo_phy_init,
	.exit = qrk3588_combo_phy_exit,
	.power_on = qrk3588_combo_phy_power_on,
	.set_mode = qrk3588_combo_phy_set_mode,
	.owner = THIS_MODULE,
};

static int qrk3588_combo_phy_probe(struct platform_device *pdev)
{
	static const char * const reset_names[] = { "grf", "apb", "core" };
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct qrk3588_combo_phy *qphy;
	struct phy *phy;
	int i;

	qphy = devm_kzalloc(dev, sizeof(*qphy), GFP_KERNEL);
	if (!qphy)
		return -ENOMEM;
	qphy->dev = dev;
	qphy->base = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(qphy->base))
		return PTR_ERR(qphy->base);
	qphy->grf = devm_platform_ioremap_resource_byname(pdev, "grf");
	if (IS_ERR(qphy->grf))
		return PTR_ERR(qphy->grf);

	if (of_property_present(dev->of_node, "quest,php-grf")) {
		qphy->php_grf =
			syscon_regmap_lookup_by_phandle(dev->of_node, "quest,php-grf");
		if (IS_ERR(qphy->php_grf))
			return dev_err_probe(dev, PTR_ERR(qphy->php_grf),
					     "failed to get PHP routing syscon\n");
	}

	qphy->num_clks = devm_clk_bulk_get_all(dev, &qphy->clks);
	if (qphy->num_clks < 0)
		return dev_err_probe(dev, qphy->num_clks, "failed to get clocks\n");
	qphy->refclk = devm_clk_get(dev, "ref");
	if (IS_ERR(qphy->refclk))
		return dev_err_probe(dev, PTR_ERR(qphy->refclk),
				     "failed to get reference clock\n");

	for (i = 0; i < ARRAY_SIZE(qphy->resets); i++) {
		qphy->resets[i].id = reset_names[i];
		qphy->resets[i].rstc =
			devm_reset_control_get_exclusive(dev, reset_names[i]);
		if (IS_ERR(qphy->resets[i].rstc))
			return dev_err_probe(dev, PTR_ERR(qphy->resets[i].rstc),
					     "failed to get %s reset\n", reset_names[i]);
	}

	phy = devm_phy_create(dev, NULL, &qrk3588_combo_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, qphy);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id qrk3588_combo_phy_of_match[] = {
	{ .compatible = "quest,qrk3588-combo-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, qrk3588_combo_phy_of_match);

static struct platform_driver qrk3588_combo_phy_driver = {
	.probe = qrk3588_combo_phy_probe,
	.driver = {
		.name = "qrk3588-combo-phy",
		.of_match_table = qrk3588_combo_phy_of_match,
	},
};
module_platform_driver(qrk3588_combo_phy_driver);

MODULE_DESCRIPTION("Quest QRK3588 Combo PIPE PHY in PCIe mode");
MODULE_LICENSE("GPL");
