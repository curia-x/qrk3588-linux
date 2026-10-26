// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 ninozhang001@gmail.com */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "sdhci-pltfm.h"

/* RK3588 TRM Part 1, SYS_GRF_SOC_CON10. */
#define SYS_GRF_SOC_CON10	0x0328
#define EMMC_CLK_BYPASS		BIT(9)
#define EMMC_CLK_STABLE		BIT(8)

/* RK3588 TRM Part 2, chapter 4. */
#define EMMC_CTRL		0x052c
#define EMMC_CARD_IS_EMMC	BIT(0)
#define EMMC_RST_N		BIT(2)
#define EMMC_DLL_CTRL		0x0800
#define EMMC_DLL_RXCLK		0x0804
#define EMMC_DLL_TXCLK		0x0808
#define EMMC_DLL_STRBIN		0x080c
#define EMMC_DLL_CMDOUT		0x0810
#define EMMC_DLL_BYPASS		BIT(24)

#define QRK3588_MAX_CLOCK	24000000U
/* xin24m / 64: the lowest rate of the CRU card-clock divider. */
#define QRK3588_MIN_CLOCK	375000U
#define QRK3588_NUM_CLOCKS	5

enum qrk3588_clock {
	QRK3588_CARD_CLOCK,
	QRK3588_AHB_CLOCK,
	QRK3588_AXI_CLOCK,
	QRK3588_BASE_CLOCK,
	QRK3588_TIMER_CLOCK,
};

struct qrk3588_mmc {
	struct clk_bulk_data clocks[QRK3588_NUM_CLOCKS];
	struct regmap *grf;
};

static struct qrk3588_mmc *qrk3588_mmc_priv(struct sdhci_host *host)
{
	return sdhci_pltfm_priv(sdhci_priv(host));
}

static unsigned int qrk3588_max_clock(struct sdhci_host *host)
{
	return QRK3588_MAX_CLOCK;
}

static unsigned int qrk3588_min_clock(struct sdhci_host *host)
{
	return QRK3588_MIN_CLOCK;
}

static unsigned int qrk3588_timeout_clock(struct sdhci_host *host)
{
	struct qrk3588_mmc *priv = qrk3588_mmc_priv(host);

	return clk_get_rate(priv->clocks[QRK3588_TIMER_CLOCK].clk);
}

static void qrk3588_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct qrk3588_mmc *priv = qrk3588_mmc_priv(host);
	struct clk *card_clock = priv->clocks[QRK3588_CARD_CLOCK].clk;
	unsigned long rate;
	int ret;

	host->mmc->actual_clock = 0;
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	if (!clock)
		return;

	/* CCLK_EMMC controls the pin rate; do not divide it a second time. */
	clock = clamp(clock, QRK3588_MIN_CLOCK, QRK3588_MAX_CLOCK);
	ret = clk_set_rate(card_clock, clock);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "cannot set card clock: %d\n", ret);
		return;
	}
	rate = clk_get_rate(card_clock);
	if (!rate || rate > clock) {
		dev_err(mmc_dev(host->mmc), "invalid card clock: %lu Hz\n", rate);
		return;
	}

	sdhci_enable_clk(host, 0);
	if (sdhci_readw(host, SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_CARD_EN)
		host->mmc->actual_clock = rate;
}

static void qrk3588_reset(struct sdhci_host *host, u8 mask)
{
	sdhci_reset(host, mask);
	if (!(mask & SDHCI_RESET_ALL))
		return;

	/* Discard any boot-loader high-speed delay configuration. */
	sdhci_writel(host, EMMC_DLL_BYPASS, EMMC_DLL_CTRL);
	sdhci_writel(host, 0, EMMC_DLL_RXCLK);
	sdhci_writel(host, 0, EMMC_DLL_TXCLK);
	sdhci_writel(host, 0, EMMC_DLL_STRBIN);
	sdhci_writel(host, 0, EMMC_DLL_CMDOUT);
	sdhci_writew(host, EMMC_RST_N | EMMC_CARD_IS_EMMC, EMMC_CTRL);
}

static const struct sdhci_ops qrk3588_sdhci_ops = {
	.set_clock = qrk3588_set_clock,
	.get_max_clock = qrk3588_max_clock,
	.get_min_clock = qrk3588_min_clock,
	.get_timeout_clock = qrk3588_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.reset = qrk3588_reset,
};

static const struct sdhci_pltfm_data qrk3588_sdhci_data = {
	.ops = &qrk3588_sdhci_ops,
	/* Initial delivery uses SDMA; ADMA has not been validated. */
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN | SDHCI_QUIRK_BROKEN_ADMA,
	/* The integrated AXI master has a 32-bit address bus (TRM 4.1). */
	.quirks2 = SDHCI_QUIRK2_BROKEN_64_BIT_DMA,
};

static int qrk3588_clock_gate(struct qrk3588_mmc *priv)
{
	/* HIWORD enables: let SDHCI gate the pin, assert external stability. */
	return regmap_write(priv->grf, SYS_GRF_SOC_CON10,
		((EMMC_CLK_BYPASS | EMMC_CLK_STABLE) << 16) | EMMC_CLK_STABLE);
}

static int qrk3588_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qrk3588_mmc *priv;
	struct sdhci_host *host;
	int ret;

	host = sdhci_pltfm_init(pdev, &qrk3588_sdhci_data, sizeof(*priv));
	if (IS_ERR(host))
		return PTR_ERR(host);
	priv = qrk3588_mmc_priv(host);
	priv->clocks[QRK3588_CARD_CLOCK].id = "card";
	priv->clocks[QRK3588_AHB_CLOCK].id = "ahb";
	priv->clocks[QRK3588_AXI_CLOCK].id = "axi";
	priv->clocks[QRK3588_BASE_CLOCK].id = "base";
	priv->clocks[QRK3588_TIMER_CLOCK].id = "timer";
	ret = devm_clk_bulk_get(dev, QRK3588_NUM_CLOCKS, priv->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get eMMC clocks\n");
	priv->grf = syscon_regmap_lookup_by_phandle(dev->of_node, "quest,sys-grf");
	if (IS_ERR(priv->grf))
		return dev_err_probe(dev, PTR_ERR(priv->grf), "cannot get SYS_GRF\n");
	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;
	sdhci_get_property(pdev);
	/* Fixed 1.8 V VCCQ also applies to legacy/HS SDR, without UHS. */
	host->flags |= SDHCI_SIGNALING_180;

	ret = clk_bulk_prepare_enable(QRK3588_NUM_CLOCKS, priv->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable eMMC clocks\n");
	/* Gate before changing the SoC clock path, including after U-Boot. */
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	ret = qrk3588_clock_gate(priv);
	if (ret)
		goto disable_clocks;

	/* No tuning/DLL or DDR support in this low-frequency implementation. */
	sdhci_read_caps(host);
	host->caps1 &= ~(SDHCI_SUPPORT_SDR50 | SDHCI_SUPPORT_SDR104 |
			 SDHCI_SUPPORT_DDR50 | SDHCI_SUPPORT_HS400);
	host->mmc->caps &= ~(MMC_CAP_UHS | MMC_CAP_DDR);
	host->mmc->caps2 &= ~(MMC_CAP2_HS200 | MMC_CAP2_HS400 | MMC_CAP2_HS400_ES);
	host->mmc->f_max = min(host->mmc->f_max ?: QRK3588_MAX_CLOCK,
			       QRK3588_MAX_CLOCK);
	ret = sdhci_add_host(host);
	if (!ret)
		return 0;

disable_clocks:
	clk_bulk_disable_unprepare(QRK3588_NUM_CLOCKS, priv->clocks);
	return ret;
}

static void qrk3588_mmc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct qrk3588_mmc *priv = qrk3588_mmc_priv(host);

	sdhci_pltfm_remove(pdev);
	clk_bulk_disable_unprepare(QRK3588_NUM_CLOCKS, priv->clocks);
}

static int qrk3588_mmc_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct qrk3588_mmc *priv = qrk3588_mmc_priv(host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (!ret)
		clk_bulk_disable_unprepare(QRK3588_NUM_CLOCKS, priv->clocks);
	return ret;
}

static int qrk3588_mmc_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct qrk3588_mmc *priv = qrk3588_mmc_priv(host);
	int ret;

	ret = clk_bulk_prepare_enable(QRK3588_NUM_CLOCKS, priv->clocks);
	if (ret)
		return ret;
	ret = qrk3588_clock_gate(priv);
	if (!ret)
		ret = sdhci_resume_host(host);
	if (ret)
		clk_bulk_disable_unprepare(QRK3588_NUM_CLOCKS, priv->clocks);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qrk3588_mmc_pm_ops, qrk3588_mmc_suspend,
			       qrk3588_mmc_resume);

static const struct of_device_id qrk3588_mmc_match[] = {
	{ .compatible = "quest,qrk3588-emmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, qrk3588_mmc_match);

static struct platform_driver qrk3588_mmc_driver = {
	.probe = qrk3588_mmc_probe,
	.remove = qrk3588_mmc_remove,
	.driver = {
		.name = "sdhci-qrk3588",
		.of_match_table = qrk3588_mmc_match,
		.pm = pm_sleep_ptr(&qrk3588_mmc_pm_ops),
	},
};
module_platform_driver(qrk3588_mmc_driver);

MODULE_DESCRIPTION("Quest QRK3588 eMMC SDHCI driver");
MODULE_LICENSE("GPL");
