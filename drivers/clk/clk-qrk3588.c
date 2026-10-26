// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 ninozhang001@gmail.com */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/quest,qrk3588-cru.h>
#include <dt-bindings/reset/quest,qrk3588-cru.h>

#define XIN24M_RATE		24000000UL

#define CRU_CPLL_CON0		0x01a0
#define CRU_CPLL_CON1		0x01a4
#define CRU_CPLL_CON2		0x01a8
#define CRU_CPLL_CON6		0x01b8
#define CRU_GPLL_CON0		0x01c0
#define CRU_GPLL_CON1		0x01c4
#define CRU_GPLL_CON2		0x01c8
#define CRU_GPLL_CON6		0x01d8
#define CRU_MODE_CON00		0x0280
#define CRU_CLKSEL_CON00	0x0300
#define CRU_CLKSEL_CON01	0x0304
#define CRU_CLKSEL_CON08	0x0320
#define CRU_GATE_CON12		0x0830
#define CRU_GATE_CON13		0x0834
#define CRU_CLKSEL_CON77	0x0434
#define CRU_CLKSEL_CON78	0x0438
#define CRU_CLKSEL_CON80	0x0440
#define CRU_CLKSEL_CON176	0x05c0
#define CRU_CLKSEL_CON177	0x05c4
#define CRU_GATE_CON00		0x0800
#define CRU_GATE_CON01		0x0804
#define CRU_GATE_CON16		0x0840
#define CRU_GATE_CON17		0x0844
#define CRU_GATE_CON31		0x087c
#define CRU_GATE_CON32		0x0880
#define CRU_GATE_CON33		0x0884
#define CRU_GATE_CON34		0x0888
#define CRU_GATE_CON37		0x0894
#define CRU_GATE_CON38		0x0898
#define CRU_GATE_CON77		0x0934
#define CRU_SOFTRST_CON32	0x0a80
#define CRU_SOFTRST_CON33	0x0a84
#define CRU_SOFTRST_CON34	0x0a88
#define CRU_SOFTRST_CON77	0x0b34

#define PHPTOP_PPLL_CON0	0x0200
#define PHPTOP_PPLL_CON1	0x0204
#define PHPTOP_PPLL_CON2	0x0208
#define PHPTOP_PPLL_CON6	0x0218
#define PHPTOP_GATE_CON00	0x0800
#define PHPTOP_SOFTRST_CON00	0x0a00
#define PMU1_GATE_CON05		0x0814

#define PLL_BYPASS		BIT(15)
#define PLL_M_MASK		GENMASK(9, 0)
#define PLL_P_MASK		GENMASK(5, 0)
#define PLL_S_MASK		GENMASK(8, 6)
#define PLL_LOCK		BIT(15)

#define QRK_GATE_FLAGS		(CLK_GATE_SET_TO_DISABLE | CLK_GATE_HIWORD_MASK)

enum qrk3588_domain {
	QRK_CRU,
	QRK_PHPTOP,
	QRK_PMU1,
};

struct qrk3588_cru {
	void __iomem *base[3];
	/* Serializes HIWORD-mask writes to shared CRU registers. */
	spinlock_t lock;
	struct clk_hw_onecell_data *data;
	struct reset_controller_dev rcdev;
};

struct qrk3588_reset_desc {
	u8 domain;
	u16 reg;
	u16 mask;
};

static const struct qrk3588_reset_desc qrk3588_resets[] = {
	[QRK3588_RESET_PCIE_FABRIC_MAIN] = {
		QRK_CRU, CRU_SOFTRST_CON32, BIT(12) | BIT(9) | BIT(8) | BIT(5)
	},
	[QRK3588_RESET_PCIE_FABRIC_AUX] = {
		QRK_CRU, CRU_SOFTRST_CON34, BIT(9)
	},
	[QRK3588_RESET_PCIE1L0_PIPE] = {
		QRK_CRU, CRU_SOFTRST_CON32, BIT(15)
	},
	[QRK3588_RESET_PCIE1L0_POWER_UP] = {
		QRK_CRU, CRU_SOFTRST_CON33, BIT(14)
	},
	[QRK3588_RESET_PCIE1L2_PIPE] = {
		QRK_CRU, CRU_SOFTRST_CON33, BIT(1)
	},
	[QRK3588_RESET_PCIE1L2_POWER_UP] = {
		QRK_CRU, CRU_SOFTRST_CON34, BIT(0)
	},
	[QRK3588_RESET_PIPE_PHY0_GRF] = {
		QRK_PHPTOP, PHPTOP_SOFTRST_CON00, BIT(2)
	},
	[QRK3588_RESET_PIPE_PHY0_APB] = {
		QRK_PHPTOP, PHPTOP_SOFTRST_CON00, BIT(5)
	},
	[QRK3588_RESET_PIPE_PHY0_CORE] = {
		QRK_CRU, CRU_SOFTRST_CON77, BIT(6)
	},
	[QRK3588_RESET_PIPE_PHY1_GRF] = {
		QRK_PHPTOP, PHPTOP_SOFTRST_CON00, BIT(3)
	},
	[QRK3588_RESET_PIPE_PHY1_APB] = {
		QRK_PHPTOP, PHPTOP_SOFTRST_CON00, BIT(6)
	},
	[QRK3588_RESET_PIPE_PHY1_CORE] = {
		QRK_CRU, CRU_SOFTRST_CON77, BIT(7)
	},
};

static int qrk3588_reset_update(struct reset_controller_dev *rcdev,
				unsigned long id, bool assert)
{
	struct qrk3588_cru *cru = container_of(rcdev, struct qrk3588_cru, rcdev);
	const struct qrk3588_reset_desc *reset;
	u32 value;

	if (id >= ARRAY_SIZE(qrk3588_resets))
		return -EINVAL;

	reset = &qrk3588_resets[id];
	value = (u32)reset->mask << 16;
	if (assert)
		value |= reset->mask;

	writel(value, cru->base[reset->domain] + reset->reg);
	return 0;
}

static int qrk3588_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return qrk3588_reset_update(rcdev, id, true);
}

static int qrk3588_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	return qrk3588_reset_update(rcdev, id, false);
}

static const struct reset_control_ops qrk3588_reset_ops = {
	.assert = qrk3588_reset_assert,
	.deassert = qrk3588_reset_deassert,
};

struct qrk3588_pll {
	struct clk_hw hw;
	void __iomem *base;
	u16 con0;
	u16 con1;
	u16 con2;
	u16 con6;
	s8 mode_shift;
};

struct qrk3588_composite {
	struct clk_mux mux;
	struct clk_divider div;
	struct clk_gate gate;
};

struct qrk3588_gate_desc {
	u8 id;
	u8 domain;
	u16 reg;
	u8 bit;
	const char *name;
};

#define to_qrk3588_pll(_hw) container_of(_hw, struct qrk3588_pll, hw)
#define QRK_GATE(_id, _name, _domain, _reg, _bit) \
	{ .id = (_id), .name = (_name), .domain = (_domain), \
	  .reg = (_reg), .bit = (_bit) }

static unsigned long qrk3588_pll_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct qrk3588_pll *pll = to_qrk3588_pll(hw);
	u32 con0, con1, mode, m, p, s;
	s64 scaled_m;
	s16 k;

	if (pll->mode_shift >= 0) {
		mode = (readl(pll->base + CRU_MODE_CON00) >> pll->mode_shift) & 0x3;
		if (mode == 0)
			return XIN24M_RATE;
		if (mode != 1)
			return 0;
	}

	con0 = readl(pll->base + pll->con0);
	if (con0 & PLL_BYPASS)
		return XIN24M_RATE;
	if (!(readl(pll->base + pll->con6) & PLL_LOCK))
		return 0;

	con1 = readl(pll->base + pll->con1);
	m = FIELD_GET(PLL_M_MASK, con0);
	p = FIELD_GET(PLL_P_MASK, con1);
	s = FIELD_GET(PLL_S_MASK, con1);
	k = (s16)readl(pll->base + pll->con2);
	if (m < 64 || !p || p > 63 || s > 6)
		return 0;

	scaled_m = ((s64)m << 16) + k;
	if (scaled_m <= 0)
		return 0;

	return div64_s64((s64)XIN24M_RATE * scaled_m,
			 ((s64)p << s) << 16);
}

static const struct clk_ops qrk3588_pll_ops = {
	.recalc_rate = qrk3588_pll_recalc_rate,
};

static struct clk_hw *qrk3588_register_pll(struct device *dev, const char *name,
					  void __iomem *base, u16 con0,
					  u16 con1, u16 con2, u16 con6,
					  s8 mode_shift)
{
	struct clk_init_data init = {
		.name = name,
		.ops = &qrk3588_pll_ops,
	};
	struct qrk3588_pll *pll;
	int ret;

	pll = devm_kzalloc(dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);
	pll->base = base;
	pll->con0 = con0;
	pll->con1 = con1;
	pll->con2 = con2;
	pll->con6 = con6;
	pll->mode_shift = mode_shift;
	pll->hw.init = &init;

	ret = devm_clk_hw_register(dev, &pll->hw);
	return ret ? ERR_PTR(ret) : &pll->hw;
}

static struct clk_hw *
qrk3588_register_composite(struct device *dev, struct qrk3588_cru *cru,
			   const char *name, struct clk_hw **parents,
			   unsigned int num_parents, u16 mux_reg,
			   u8 mux_shift, u8 mux_width, u16 div_reg,
			   u8 div_shift, u8 div_width, u16 gate_reg,
			   u8 gate_bit, unsigned long flags)
{
	struct clk_parent_data *parent_data;
	struct qrk3588_composite *comp;
	struct clk_hw *mux_hw = NULL, *div_hw = NULL, *gate_hw = NULL;
	const struct clk_ops *mux_ops = NULL, *div_ops = NULL, *gate_ops = NULL;
	unsigned int i;

	comp = devm_kzalloc(dev, sizeof(*comp), GFP_KERNEL);
	parent_data = devm_kcalloc(dev, num_parents, sizeof(*parent_data),
				  GFP_KERNEL);
	if (!comp || !parent_data)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num_parents; i++)
		parent_data[i].hw = parents[i];

	if (mux_width) {
		comp->mux.reg = cru->base[QRK_CRU] + mux_reg;
		comp->mux.shift = mux_shift;
		comp->mux.mask = BIT(mux_width) - 1;
		comp->mux.flags = CLK_MUX_HIWORD_MASK;
		comp->mux.lock = &cru->lock;
		mux_hw = &comp->mux.hw;
		mux_ops = &clk_mux_ops;
	}
	if (div_width) {
		comp->div.reg = cru->base[QRK_CRU] + div_reg;
		comp->div.shift = div_shift;
		comp->div.width = div_width;
		comp->div.flags = CLK_DIVIDER_HIWORD_MASK;
		comp->div.lock = &cru->lock;
		div_hw = &comp->div.hw;
		div_ops = &clk_divider_ops;
	}
	if (gate_reg) {
		comp->gate.reg = cru->base[QRK_CRU] + gate_reg;
		comp->gate.bit_idx = gate_bit;
		comp->gate.flags = QRK_GATE_FLAGS;
		comp->gate.lock = &cru->lock;
		gate_hw = &comp->gate.hw;
		gate_ops = &clk_gate_ops;
	}

	return devm_clk_hw_register_composite_pdata(dev, name, parent_data,
				 num_parents, mux_hw, mux_ops, div_hw, div_ops,
				 gate_hw, gate_ops, flags);
}

static const struct qrk3588_gate_desc qrk3588_pcie_gates[] = {
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_BIU, "aclk_pcie_biu", QRK_CRU, CRU_GATE_CON32, 12),
	QRK_GATE(QRK3588_CLK_ACLK_PHP_BIU, "aclk_php_biu", QRK_CRU, CRU_GATE_CON32, 9),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_BRIDGE, "aclk_pcie_bridge", QRK_CRU, CRU_GATE_CON32, 8),
	QRK_GATE(QRK3588_CLK_PCLK_PHP_BIU, "pclk_php_biu", QRK_CRU, CRU_GATE_CON32, 5),
	QRK_GATE(QRK3588_CLK_PCLK_PHP_ROOT, "pclk_php_root", QRK_CRU, CRU_GATE_CON32, 0),
	QRK_GATE(QRK3588_CLK_ACLK_MMU_BIU, "aclk_mmu_biu", QRK_CRU, CRU_GATE_CON34, 9),
	QRK_GATE(QRK3588_CLK_ACLK_MMU_PCIE, "aclk_mmu_pcie", QRK_CRU, CRU_GATE_CON34, 7),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_1L0_DBI, "aclk_pcie_1l0_dbi", QRK_CRU, CRU_GATE_CON32, 15),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_1L0_SLV, "aclk_pcie_1l0_slv", QRK_CRU, CRU_GATE_CON33, 9),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_1L0_MSTR, "aclk_pcie_1l0_mstr", QRK_CRU, CRU_GATE_CON33, 4),
	QRK_GATE(QRK3588_CLK_PCIE_1L0_AUX, "clk_pcie_1l0_aux", QRK_CRU, CRU_GATE_CON34, 3),
	QRK_GATE(QRK3588_CLK_PCIE_1L0_PIPE, "clk_pcie_1l0_pipe", QRK_CRU, CRU_GATE_CON38, 14),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_1L2_SLV, "aclk_pcie_1l2_slv", QRK_CRU, CRU_GATE_CON33, 11),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_1L2_MSTR, "aclk_pcie_1l2_mstr", QRK_CRU, CRU_GATE_CON33, 6),
	QRK_GATE(QRK3588_CLK_ACLK_PCIE_1L2_DBI, "aclk_pcie_1l2_dbi", QRK_CRU, CRU_GATE_CON33, 1),
	QRK_GATE(QRK3588_CLK_PCIE_1L2_AUX, "clk_pcie_1l2_aux", QRK_CRU, CRU_GATE_CON34, 5),
	QRK_GATE(QRK3588_CLK_PCIE_1L2_PIPE, "clk_pcie_1l2_pipe", QRK_CRU, CRU_GATE_CON38, 13),
	QRK_GATE(QRK3588_CLK_PIPEPHY0_REF, "clk_pipephy0_ref", QRK_CRU, CRU_GATE_CON37, 0),
	QRK_GATE(QRK3588_CLK_PMALIVE0, "clk_pmalive0", QRK_CRU, CRU_GATE_CON37, 4),
	QRK_GATE(QRK3588_CLK_PIPEPHY0_PIPE, "clk_pipephy0_pipe", QRK_CRU, CRU_GATE_CON38, 3),
	QRK_GATE(QRK3588_CLK_PIPEPHY0_PIPE_ASIC, "clk_pipephy0_pipe_asic",
		 QRK_CRU, CRU_GATE_CON38, 6),
	QRK_GATE(QRK3588_CLK_PCLK_PIPEPHY0_GRF, "pclk_pipephy0_grf",
		 QRK_PHPTOP, PHPTOP_GATE_CON00, 2),
	QRK_GATE(QRK3588_CLK_PIPEPHY1_REF, "clk_pipephy1_ref", QRK_CRU, CRU_GATE_CON37, 1),
	QRK_GATE(QRK3588_CLK_PMALIVE1, "clk_pmalive1", QRK_CRU, CRU_GATE_CON37, 5),
	QRK_GATE(QRK3588_CLK_PIPEPHY1_PIPE, "clk_pipephy1_pipe", QRK_CRU, CRU_GATE_CON38, 4),
	QRK_GATE(QRK3588_CLK_PIPEPHY1_PIPE_ASIC, "clk_pipephy1_pipe_asic",
		 QRK_CRU, CRU_GATE_CON38, 7),
	QRK_GATE(QRK3588_CLK_PCLK_PIPEPHY1_GRF, "pclk_pipephy1_grf",
		 QRK_PHPTOP, PHPTOP_GATE_CON00, 3),
};

static int qrk3588_register_pcie_gates(struct device *dev,
				       struct qrk3588_cru *cru)
{
	const struct qrk3588_gate_desc *gate;
	struct clk_hw *hw;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(qrk3588_pcie_gates); i++) {
		gate = &qrk3588_pcie_gates[i];
		hw = devm_clk_hw_register_gate(dev, gate->name, NULL, 0,
			cru->base[gate->domain] + gate->reg, gate->bit,
			QRK_GATE_FLAGS, &cru->lock);
		if (IS_ERR(hw))
			return dev_err_probe(dev, PTR_ERR(hw),
					     "failed to register %s\n", gate->name);
		cru->data->hws[gate->id] = hw;
	}

	return 0;
}

static int qrk3588_cru_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qrk3588_cru *cru;
	struct clk_hw *xin, *cpll, *gpll, *ppll;
	struct clk_hw *matrix50, *matrix100, *matrix200;
	struct clk_hw *pclk_top_root;
	struct clk_hw *hclk_root, *aclk_root, *hclk_biu, *aclk_biu;
	struct clk_hw *aclk_pcie_root, *aclk_php_root;
	struct clk_hw *osc0, *osc1, *pll0, *pll1;
	struct clk_hw *parents[4];
	const struct clk_hw *mux_parents[2];
	unsigned int i;
	int ret;

	cru = devm_kzalloc(dev, sizeof(*cru), GFP_KERNEL);
	if (!cru)
		return -ENOMEM;
	cru->base[QRK_CRU] = devm_platform_ioremap_resource_byname(pdev, "cru");
	if (IS_ERR(cru->base[QRK_CRU]))
		return PTR_ERR(cru->base[QRK_CRU]);
	cru->base[QRK_PHPTOP] = devm_platform_ioremap_resource_byname(pdev,
							     "phptop-cru");
	if (IS_ERR(cru->base[QRK_PHPTOP]))
		return PTR_ERR(cru->base[QRK_PHPTOP]);
	cru->base[QRK_PMU1] = devm_platform_ioremap_resource_byname(pdev,
							  "pmu1-cru");
	if (IS_ERR(cru->base[QRK_PMU1]))
		return PTR_ERR(cru->base[QRK_PMU1]);
	spin_lock_init(&cru->lock);

	cru->data = devm_kzalloc(dev, struct_size(cru->data, hws,
						 QRK3588_CLK_COUNT), GFP_KERNEL);
	if (!cru->data)
		return -ENOMEM;
	cru->data->num = QRK3588_CLK_COUNT;

	xin = devm_clk_hw_register_fixed_rate(dev, "xin24m", NULL, 0,
					       XIN24M_RATE);
	cpll = qrk3588_register_pll(dev, "cpll", cru->base[QRK_CRU],
		CRU_CPLL_CON0, CRU_CPLL_CON1, CRU_CPLL_CON2, CRU_CPLL_CON6, 8);
	gpll = qrk3588_register_pll(dev, "gpll", cru->base[QRK_CRU],
		CRU_GPLL_CON0, CRU_GPLL_CON1, CRU_GPLL_CON2, CRU_GPLL_CON6, 2);
	ppll = qrk3588_register_pll(dev, "ppll", cru->base[QRK_PHPTOP],
		PHPTOP_PPLL_CON0, PHPTOP_PPLL_CON1, PHPTOP_PPLL_CON2,
		PHPTOP_PPLL_CON6, -1);
	if (IS_ERR(xin))
		return PTR_ERR(xin);
	if (IS_ERR(cpll))
		return PTR_ERR(cpll);
	if (IS_ERR(gpll))
		return PTR_ERR(gpll);
	if (IS_ERR(ppll))
		return PTR_ERR(ppll);

	parents[0] = gpll;
	parents[1] = cpll;
	matrix50 = qrk3588_register_composite(dev, cru, "matrix50", parents, 2,
		CRU_CLKSEL_CON00, 5, 1, CRU_CLKSEL_CON00, 0, 5,
		CRU_GATE_CON00, 0, 0);
	matrix100 = qrk3588_register_composite(dev, cru, "matrix100", parents, 2,
		CRU_CLKSEL_CON00, 11, 1, CRU_CLKSEL_CON00, 6, 5,
		CRU_GATE_CON00, 1, 0);
	matrix200 = qrk3588_register_composite(dev, cru, "matrix200", parents, 2,
		CRU_CLKSEL_CON01, 11, 1, CRU_CLKSEL_CON01, 6, 5,
		CRU_GATE_CON00, 3, 0);
	if (IS_ERR(matrix50) || IS_ERR(matrix100) || IS_ERR(matrix200))
		return -EINVAL;
	parents[0] = matrix100;
	parents[1] = matrix50;
	parents[2] = xin;
	pclk_top_root = qrk3588_register_composite(dev, cru, "pclk_top_root",
						   parents, 3, CRU_CLKSEL_CON08,
						   7, 2, 0, 0, 0,
						   CRU_GATE_CON01, 1, 0);
	if (IS_ERR(pclk_top_root))
		return PTR_ERR(pclk_top_root);

	parents[0] = matrix200;
	parents[1] = matrix100;
	parents[2] = matrix50;
	parents[3] = xin;
	hclk_root = qrk3588_register_composite(dev, cru, "hclk_nvm_root",
		parents, 4, CRU_CLKSEL_CON77, 0, 2, 0, 0, 0,
		CRU_GATE_CON31, 0, 0);
	parents[0] = gpll;
	parents[1] = cpll;
	aclk_root = qrk3588_register_composite(dev, cru, "aclk_nvm_root",
		parents, 2, CRU_CLKSEL_CON77, 7, 1, CRU_CLKSEL_CON77, 2, 5,
		CRU_GATE_CON31, 1, 0);
	if (IS_ERR(hclk_root) || IS_ERR(aclk_root))
		return -EINVAL;

	hclk_biu = devm_clk_hw_register_gate_parent_hw(dev, "hclk_nvm_biu",
		hclk_root, 0, cru->base[QRK_CRU] + CRU_GATE_CON31, 2,
		QRK_GATE_FLAGS, &cru->lock);
	aclk_biu = devm_clk_hw_register_gate_parent_hw(dev, "aclk_nvm_biu",
		aclk_root, 0, cru->base[QRK_CRU] + CRU_GATE_CON31, 3,
		QRK_GATE_FLAGS, &cru->lock);
	if (IS_ERR(hclk_biu) || IS_ERR(aclk_biu))
		return -EINVAL;

	parents[0] = gpll;
	parents[1] = cpll;
	parents[2] = xin;
	cru->data->hws[QRK3588_CLK_CCLK_EMMC] = qrk3588_register_composite(dev,
		cru, "cclk_emmc", parents, 3, CRU_CLKSEL_CON77, 14, 2,
		CRU_CLKSEL_CON77, 8, 6, CRU_GATE_CON31, 6, 0);
	parents[0] = gpll;
	parents[1] = cpll;
	cru->data->hws[QRK3588_CLK_BCLK_EMMC] = qrk3588_register_composite(dev,
		cru, "bclk_emmc", parents, 2, CRU_CLKSEL_CON78, 5, 1,
		CRU_CLKSEL_CON78, 0, 5, CRU_GATE_CON31, 7, 0);
	cru->data->hws[QRK3588_CLK_HCLK_EMMC] =
		devm_clk_hw_register_gate_parent_hw(dev, "hclk_emmc", hclk_biu, 0,
		cru->base[QRK_CRU] + CRU_GATE_CON31, 4,
		QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_ACLK_EMMC] =
		devm_clk_hw_register_gate_parent_hw(dev, "aclk_emmc", aclk_biu, 0,
		cru->base[QRK_CRU] + CRU_GATE_CON31, 5,
		QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_TMCLK_EMMC] =
		devm_clk_hw_register_gate_parent_hw(dev, "tmclk_emmc", xin, 0,
		cru->base[QRK_CRU] + CRU_GATE_CON31, 8,
		QRK_GATE_FLAGS, &cru->lock);

	cru->data->hws[QRK3588_CLK_PCLK_GPIO0] = devm_clk_hw_register_gate(dev,
		"pclk_gpio0", NULL, 0, cru->base[QRK_PMU1] + PMU1_GATE_CON05,
		5, QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_GPIO1] = devm_clk_hw_register_gate(dev,
		"pclk_gpio1", NULL, 0, cru->base[QRK_CRU] + CRU_GATE_CON16,
		14, QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_GPIO2] = devm_clk_hw_register_gate(dev,
		"pclk_gpio2", NULL, 0, cru->base[QRK_CRU] + CRU_GATE_CON17,
		0, QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_GPIO3] = devm_clk_hw_register_gate(dev,
		"pclk_gpio3", NULL, 0, cru->base[QRK_CRU] + CRU_GATE_CON17,
		2, QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_GPIO4] = devm_clk_hw_register_gate(dev,
		"pclk_gpio4", NULL, 0, cru->base[QRK_CRU] + CRU_GATE_CON17,
		4, QRK_GATE_FLAGS, &cru->lock);

	/* UART2 reset state selects xin24m as its serial clock source. */
	cru->data->hws[QRK3588_CLK_SCLK_UART2] =
		devm_clk_hw_register_gate_parent_hw(dev, "sclk_uart2", xin, 0,
			cru->base[QRK_CRU] + CRU_GATE_CON13, 0,
			QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_UART2] =
		devm_clk_hw_register_gate_parent_hw(dev, "pclk_uart2",
						    pclk_top_root, 0,
						    cru->base[QRK_CRU] + CRU_GATE_CON12,
						    3, QRK_GATE_FLAGS, &cru->lock);

	osc0 = devm_clk_hw_register_gate_parent_hw(dev, "ref_pipe_phy0_osc", xin,
		0, cru->base[QRK_CRU] + CRU_GATE_CON77, 0,
		QRK_GATE_FLAGS, &cru->lock);
	osc1 = devm_clk_hw_register_gate_parent_hw(dev, "ref_pipe_phy1_osc", xin,
		0, cru->base[QRK_CRU] + CRU_GATE_CON77, 1,
		QRK_GATE_FLAGS, &cru->lock);
	parents[0] = ppll;
	pll0 = qrk3588_register_composite(dev, cru, "ref_pipe_phy0_pll", parents,
		1, 0, 0, 0, CRU_CLKSEL_CON176, 0, 6, CRU_GATE_CON77, 3, 0);
	pll1 = qrk3588_register_composite(dev, cru, "ref_pipe_phy1_pll", parents,
		1, 0, 0, 0, CRU_CLKSEL_CON176, 6, 6, CRU_GATE_CON77, 4, 0);
	if (IS_ERR(osc0) || IS_ERR(osc1) || IS_ERR(pll0) || IS_ERR(pll1))
		return -EINVAL;
	mux_parents[0] = osc0;
	mux_parents[1] = pll0;
	cru->data->hws[QRK3588_CLK_REF_PIPE_PHY0] =
		devm_clk_hw_register_mux_parent_hws(dev, "ref_pipe_phy0",
		mux_parents, 2,
		CLK_SET_RATE_PARENT, cru->base[QRK_CRU] + CRU_CLKSEL_CON177,
		6, 1, CLK_MUX_HIWORD_MASK, &cru->lock);
	mux_parents[0] = osc1;
	mux_parents[1] = pll1;
	cru->data->hws[QRK3588_CLK_REF_PIPE_PHY1] =
		devm_clk_hw_register_mux_parent_hws(dev, "ref_pipe_phy1",
		mux_parents, 2,
		CLK_SET_RATE_PARENT, cru->base[QRK_CRU] + CRU_CLKSEL_CON177,
		7, 1, CLK_MUX_HIWORD_MASK, &cru->lock);

	cru->data->hws[QRK3588_CLK_PCLK_PCIE_1L0] = devm_clk_hw_register_gate(dev,
		"pclk_pcie_1l0", NULL, 0, cru->base[QRK_CRU] + CRU_GATE_CON33,
		14, QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_PCIE_1L2] = devm_clk_hw_register_gate(dev,
		"pclk_pcie_1l2", NULL, 0, cru->base[QRK_CRU] + CRU_GATE_CON34,
		0, QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_PCIE_COMBO_PIPE_PHY0] =
		devm_clk_hw_register_gate(dev, "pclk_pcie_combo_pipe_phy0", NULL, 0,
		cru->base[QRK_PHPTOP] + PHPTOP_GATE_CON00, 5,
		QRK_GATE_FLAGS, &cru->lock);
	cru->data->hws[QRK3588_CLK_PCLK_PCIE_COMBO_PIPE_PHY1] =
		devm_clk_hw_register_gate(dev, "pclk_pcie_combo_pipe_phy1", NULL, 0,
		cru->base[QRK_PHPTOP] + PHPTOP_GATE_CON00, 6,
		QRK_GATE_FLAGS, &cru->lock);

	/*
	 * The ITS is initialized before ordinary platform drivers probe. Keep
	 * its modeled leaf and direct ACLK parent enabled after probe. Other PHP
	 * fabric gates are independent clocks; this does not cover their lifetime.
	 */
	parents[0] = gpll;
	parents[1] = cpll;
	aclk_pcie_root = qrk3588_register_composite(dev, cru, "aclk_pcie_root",
				parents, 2, CRU_CLKSEL_CON80, 7, 1,
				CRU_CLKSEL_CON80, 2, 5,
				CRU_GATE_CON32, 6, 0);
	aclk_php_root = qrk3588_register_composite(dev, cru, "aclk_php_root",
				parents, 2, CRU_CLKSEL_CON80, 13, 1,
				CRU_CLKSEL_CON80, 8, 5,
				CRU_GATE_CON32, 7, 0);
	if (IS_ERR(aclk_pcie_root) || IS_ERR(aclk_php_root))
		return -EINVAL;
	cru->data->hws[QRK3588_CLK_ACLK_PCIE_ROOT] = aclk_pcie_root;
	cru->data->hws[QRK3588_CLK_ACLK_PHP_ROOT] = aclk_php_root;
	cru->data->hws[QRK3588_CLK_ACLK_PHP_GIC_ITS] =
		devm_clk_hw_register_gate_parent_hw(dev, "aclk_php_gic_its",
					aclk_pcie_root,
					CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
					cru->base[QRK_CRU] + CRU_GATE_CON34,
					6, QRK_GATE_FLAGS, &cru->lock);

	ret = qrk3588_register_pcie_gates(dev, cru);
	if (ret)
		return ret;

	for (i = 0; i < QRK3588_CLK_COUNT; i++) {
		if (!cru->data->hws[i])
			return dev_err_probe(dev, -ENOENT,
					     "clock %u is not registered\n", i);
		if (IS_ERR(cru->data->hws[i]))
			return dev_err_probe(dev, PTR_ERR(cru->data->hws[i]),
					     "failed to register clock %u\n", i);
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, cru->data);
	if (ret)
		return ret;

	cru->rcdev.owner = THIS_MODULE;
	cru->rcdev.nr_resets = QRK3588_RESET_COUNT;
	cru->rcdev.ops = &qrk3588_reset_ops;
	cru->rcdev.of_node = dev->of_node;

	return devm_reset_controller_register(dev, &cru->rcdev);
}

static const struct of_device_id qrk3588_cru_of_match[] = {
	{ .compatible = "quest,qrk3588-cru" },
	{ }
};
MODULE_DEVICE_TABLE(of, qrk3588_cru_of_match);

static struct platform_driver qrk3588_cru_driver = {
	.probe = qrk3588_cru_probe,
	.driver = {
		.name = "qrk3588-cru",
		.of_match_table = qrk3588_cru_of_match,
	},
};
module_platform_driver(qrk3588_cru_driver);

MODULE_DESCRIPTION("Quest QRK3588 clock controller");
MODULE_LICENSE("GPL");
