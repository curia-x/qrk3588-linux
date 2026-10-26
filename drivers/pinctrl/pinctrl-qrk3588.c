// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 ninozhang001@gmail.com

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "core.h"
#include "pinmux.h"

#define QRK3588_PINS_PER_BANK	32
#define QRK3588_PIN_COUNT	(5 * QRK3588_PINS_PER_BANK)

#define PMU1_GPIO0A_IOMUX_L	0x0000
#define PMU1_GPIO0A_IOMUX_H	0x0004
#define PMU1_GPIO0B_IOMUX_L	0x0008
#define PMU2_GPIO0B_IOMUX_H	0x0000
#define PMU2_GPIO0C_IOMUX_L	0x0004
#define PMU2_GPIO0C_IOMUX_H	0x0008
#define PMU2_GPIO0D_IOMUX_L	0x000c
#define PMU2_GPIO0D_IOMUX_H	0x0010
#define BUS_GPIO0B_IOMUX_H	0x000c
#define BUS_IOMUX_BANK_STRIDE	0x0020
#define BUS_IOMUX_GROUP_STRIDE	0x0008
#define BUS_IOMUX_HALF_STRIDE	0x0004
#define BUS_GPIO2A_IOMUX_L	0x0040
#define BUS_GPIO2D_IOMUX_L	0x0058
#define BUS_GPIO2D_IOMUX_H	0x005c

#define EMMC_GPIO2A_DS_L	0x0040
#define EMMC_GPIO2D_DS_L	0x0058
#define EMMC_GPIO2D_DS_H	0x005c
#define EMMC_GPIO2A_PULL	0x0120
#define EMMC_GPIO2D_PULL	0x012c
#define EMMC_GPIO2A_IE		0x0190
#define EMMC_GPIO2D_IE		0x019c
#define EMMC_GPIO2A_SMT		0x0220
#define EMMC_GPIO2D_SMT		0x022c

#define QRK3588_EMMC_DS_40_OHM	6

enum qrk3588_mux {
	QRK3588_MUX_UART2,
	QRK3588_MUX_EMMC,
};

struct qrk3588_pin_group {
	const char *name;
	const unsigned int *pins;
	unsigned int npins;
	enum qrk3588_mux mux;
};

struct qrk3588_pin_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
	enum qrk3588_mux mux;
};

struct qrk3588_pinctrl {
	void __iomem *pmu1_ioc;
	void __iomem *pmu2_ioc;
	void __iomem *bus_ioc;
	void __iomem *emmc_ioc;
	struct pinctrl_dev *pctldev;
	u8 saved_mux[QRK3588_PIN_COUNT];
	bool saved_mux_valid[QRK3588_PIN_COUNT];
};

#define QRK3588_PIN(bank, group, group_name, index) \
	PINCTRL_PIN((bank) * QRK3588_PINS_PER_BANK + (group) * 8 + (index), \
		    "gpio" #bank "-" #group_name #index)
#define QRK3588_PIN_GROUP(bank, group, group_name) \
	QRK3588_PIN(bank, group, group_name, 0), \
	QRK3588_PIN(bank, group, group_name, 1), \
	QRK3588_PIN(bank, group, group_name, 2), \
	QRK3588_PIN(bank, group, group_name, 3), \
	QRK3588_PIN(bank, group, group_name, 4), \
	QRK3588_PIN(bank, group, group_name, 5), \
	QRK3588_PIN(bank, group, group_name, 6), \
	QRK3588_PIN(bank, group, group_name, 7)

static const struct pinctrl_pin_desc qrk3588_pins[] = {
	QRK3588_PIN_GROUP(0, 0, a),
	QRK3588_PIN_GROUP(0, 1, b),
	QRK3588_PIN_GROUP(0, 2, c),
	QRK3588_PIN_GROUP(0, 3, d),
	QRK3588_PIN_GROUP(1, 0, a),
	QRK3588_PIN_GROUP(1, 1, b),
	QRK3588_PIN_GROUP(1, 2, c),
	QRK3588_PIN_GROUP(1, 3, d),
	QRK3588_PIN_GROUP(2, 0, a),
	QRK3588_PIN_GROUP(2, 1, b),
	QRK3588_PIN_GROUP(2, 2, c),
	QRK3588_PIN_GROUP(2, 3, d),
	QRK3588_PIN_GROUP(3, 0, a),
	QRK3588_PIN_GROUP(3, 1, b),
	QRK3588_PIN_GROUP(3, 2, c),
	QRK3588_PIN_GROUP(3, 3, d),
	QRK3588_PIN_GROUP(4, 0, a),
	QRK3588_PIN_GROUP(4, 1, b),
	QRK3588_PIN_GROUP(4, 2, c),
	QRK3588_PIN_GROUP(4, 3, d),
};

static const unsigned int uart2m0_pins[] = { 13, 14 };
static const unsigned int emmc_cmd_pins[] = { 64 };
static const unsigned int emmc_clk_strobe_pins[] = { 65, 66 };
static const unsigned int emmc_rstn_pins[] = { 67 };
static const unsigned int emmc_data_pins[] = {
	88, 89, 90, 91, 92, 93, 94, 95,
};

static struct qrk3588_pin_group qrk3588_groups[] = {
	{
		.name = "uart2m0",
		.pins = uart2m0_pins,
		.npins = ARRAY_SIZE(uart2m0_pins),
		.mux = QRK3588_MUX_UART2,
	}, {
		.name = "emmc-cmd",
		.pins = emmc_cmd_pins,
		.npins = ARRAY_SIZE(emmc_cmd_pins),
		.mux = QRK3588_MUX_EMMC,
	}, {
		.name = "emmc-clk-strobe",
		.pins = emmc_clk_strobe_pins,
		.npins = ARRAY_SIZE(emmc_clk_strobe_pins),
		.mux = QRK3588_MUX_EMMC,
	}, {
		.name = "emmc-rstn",
		.pins = emmc_rstn_pins,
		.npins = ARRAY_SIZE(emmc_rstn_pins),
		.mux = QRK3588_MUX_EMMC,
	}, {
		.name = "emmc-data",
		.pins = emmc_data_pins,
		.npins = ARRAY_SIZE(emmc_data_pins),
		.mux = QRK3588_MUX_EMMC,
	},
};

static const char * const uart2_groups[] = { "uart2m0" };
static const char * const emmc_groups[] = {
	"emmc-cmd", "emmc-clk-strobe", "emmc-rstn", "emmc-data",
};

static struct qrk3588_pin_function qrk3588_functions[] = {
	{
		.name = "uart2",
		.groups = uart2_groups,
		.ngroups = ARRAY_SIZE(uart2_groups),
		.mux = QRK3588_MUX_UART2,
	}, {
		.name = "emmc",
		.groups = emmc_groups,
		.ngroups = ARRAY_SIZE(emmc_groups),
		.mux = QRK3588_MUX_EMMC,
	},
};

static void qrk3588_hiword_field_write(void __iomem *base,
				       unsigned int offset,
					unsigned int shift,
					unsigned int width,
					unsigned int value)
{
	u32 mask = GENMASK(shift + width - 1, shift);

	writel(mask << 16 | (value << shift & mask), base + offset);
}

static bool qrk3588_gpio_pin_valid(unsigned int pin)
{
	unsigned int bank = pin / QRK3588_PINS_PER_BANK;
	unsigned int offset = pin % QRK3588_PINS_PER_BANK;

	if (pin >= QRK3588_PIN_COUNT)
		return false;

	switch (bank) {
	case 0:
		return offset != 12 && offset != 31;
	case 1:
		return true;
	case 2:
		return offset != 4 && offset != 5 &&
		       offset != 22 && offset != 23;
	case 3:
		return offset < 30;
	case 4:
		return offset != 23 && offset < 30;
	default:
		return false;
	}
}

static int qrk3588_gpio_mux_location(struct qrk3588_pinctrl *pctrl,
				     unsigned int pin, void __iomem **base,
				     unsigned int *offset,
				     unsigned int *shift)
{
	unsigned int bank = pin / QRK3588_PINS_PER_BANK;
	unsigned int pin_in_bank = pin % QRK3588_PINS_PER_BANK;
	unsigned int group = pin_in_bank / 8;
	unsigned int index = pin_in_bank % 8;

	if (!qrk3588_gpio_pin_valid(pin))
		return -EINVAL;

	if (bank) {
		*base = pctrl->bus_ioc;
		*offset = bank * BUS_IOMUX_BANK_STRIDE +
			  group * BUS_IOMUX_GROUP_STRIDE +
			  (index >= 4 ? BUS_IOMUX_HALF_STRIDE : 0);
		*shift = (index & 3) * 4;
		return 0;
	}

	switch (group) {
	case 0:
		*base = pctrl->pmu1_ioc;
		*offset = index < 4 ? PMU1_GPIO0A_IOMUX_L :
				      PMU1_GPIO0A_IOMUX_H;
		break;
	case 1:
		if (index < 4) {
			*base = pctrl->pmu1_ioc;
			*offset = PMU1_GPIO0B_IOMUX_L;
		} else {
			*base = pctrl->pmu2_ioc;
			*offset = PMU2_GPIO0B_IOMUX_H;
		}
		break;
	case 2:
		*base = pctrl->pmu2_ioc;
		*offset = index < 4 ? PMU2_GPIO0C_IOMUX_L :
				      PMU2_GPIO0C_IOMUX_H;
		break;
	case 3:
		*base = pctrl->pmu2_ioc;
		*offset = index < 4 ? PMU2_GPIO0D_IOMUX_L :
				      PMU2_GPIO0D_IOMUX_H;
		break;
	default:
		return -EINVAL;
	}

	*shift = (index & 3) * 4;
	return 0;
}

static int qrk3588_gpio_mux_read(struct qrk3588_pinctrl *pctrl,
				 unsigned int pin, u8 *mux)
{
	void __iomem *base;
	unsigned int offset;
	unsigned int shift;
	int ret;

	ret = qrk3588_gpio_mux_location(pctrl, pin, &base, &offset, &shift);
	if (ret)
		return ret;

	*mux = (readl(base + offset) >> shift) & 0xf;
	return 0;
}

static int qrk3588_gpio_mux_write(struct qrk3588_pinctrl *pctrl,
				  unsigned int pin, u8 mux)
{
	void __iomem *base;
	unsigned int offset;
	unsigned int shift;
	int ret;

	ret = qrk3588_gpio_mux_location(pctrl, pin, &base, &offset, &shift);
	if (ret)
		return ret;

	qrk3588_hiword_field_write(base, offset, shift, 4, mux);
	return 0;
}

static int qrk3588_gpio_request_enable(struct pinctrl_dev *pctldev,
				       struct pinctrl_gpio_range *range,
				       unsigned int pin)
{
	struct qrk3588_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	u8 mux;
	int ret;

	ret = qrk3588_gpio_mux_read(pctrl, pin, &mux);
	if (ret)
		return ret;

	ret = qrk3588_gpio_mux_write(pctrl, pin, 0);
	if (ret)
		return ret;

	pctrl->saved_mux[pin] = mux;
	pctrl->saved_mux_valid[pin] = true;
	return 0;
}

static void qrk3588_gpio_disable_free(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int pin)
{
	struct qrk3588_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	if (pin >= QRK3588_PIN_COUNT || !pctrl->saved_mux_valid[pin])
		return;

	if (!qrk3588_gpio_mux_write(pctrl, pin, pctrl->saved_mux[pin]))
		pctrl->saved_mux_valid[pin] = false;
}

static int qrk3588_set_uart2_mux(struct qrk3588_pinctrl *pctrl,
				 unsigned int pin)
{
	unsigned int shift;

	switch (pin) {
	case 13:
		shift = 4;
		break;
	case 14:
		shift = 8;
		break;
	default:
		return -EINVAL;
	}

	/* Delegate GPIO0_B5/B6 mux selection to BUS_IOC. */
	qrk3588_hiword_field_write(pctrl->pmu2_ioc, PMU2_GPIO0B_IOMUX_H,
				   shift, 4, 8);
	qrk3588_hiword_field_write(pctrl->bus_ioc, BUS_GPIO0B_IOMUX_H,
				   shift, 4, 10);

	return 0;
}

static int qrk3588_set_emmc_mux(struct qrk3588_pinctrl *pctrl,
				unsigned int pin)
{
	unsigned int offset;
	unsigned int shift;

	if (pin >= 64 && pin <= 67) {
		offset = BUS_GPIO2A_IOMUX_L;
		shift = (pin - 64) * 4;
	} else if (pin >= 88 && pin <= 91) {
		offset = BUS_GPIO2D_IOMUX_L;
		shift = (pin - 88) * 4;
	} else if (pin >= 92 && pin <= 95) {
		offset = BUS_GPIO2D_IOMUX_H;
		shift = (pin - 92) * 4;
	} else {
		return -EINVAL;
	}

	qrk3588_hiword_field_write(pctrl->bus_ioc, offset, shift, 4, 1);

	return 0;
}

static int qrk3588_set_mux(struct pinctrl_dev *pctldev,
			   unsigned int function_selector,
			   unsigned int group_selector)
{
	const struct function_desc *function;
	const struct group_desc *group;
	const struct qrk3588_pin_function *function_data;
	const struct qrk3588_pin_group *group_data;
	struct qrk3588_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;
	int ret;

	function = pinmux_generic_get_function(pctldev, function_selector);
	group = pinctrl_generic_get_group(pctldev, group_selector);
	if (!function || !group)
		return -EINVAL;

	function_data = function->data;
	group_data = group->data;
	if (function_data->mux != group_data->mux)
		return -EINVAL;

	for (i = 0; i < group->grp.npins; i++) {
		switch (group_data->mux) {
		case QRK3588_MUX_UART2:
			ret = qrk3588_set_uart2_mux(pctrl, group->grp.pins[i]);
			break;
		case QRK3588_MUX_EMMC:
			ret = qrk3588_set_emmc_mux(pctrl, group->grp.pins[i]);
			break;
		default:
			return -EINVAL;
		}

		if (ret)
			return ret;
	}

	return 0;
}

static int qrk3588_emmc_pin_location(unsigned int pin, unsigned int *bank,
				     unsigned int *index)
{
	if (pin >= 64 && pin <= 67) {
		*bank = 0;
		*index = pin - 64;
		return 0;
	}

	if (pin >= 88 && pin <= 95) {
		*bank = 1;
		*index = pin - 88;
		return 0;
	}

	return -EOPNOTSUPP;
}

static int qrk3588_emmc_pin_config_set(struct qrk3588_pinctrl *pctrl,
				       unsigned int pin,
				       unsigned long config)
{
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 argument = pinconf_to_config_argument(config);
	unsigned int bank;
	unsigned int index;
	unsigned int offset;
	unsigned int value;
	int ret;

	ret = qrk3588_emmc_pin_location(pin, &bank, &index);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		value = 0;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		value = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		value = 3;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		value = argument;
		offset = bank ? EMMC_GPIO2D_IE : EMMC_GPIO2A_IE;
		qrk3588_hiword_field_write(pctrl->emmc_ioc, offset, index, 1,
					   value);
		return 0;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		value = argument;
		offset = bank ? EMMC_GPIO2D_SMT : EMMC_GPIO2A_SMT;
		qrk3588_hiword_field_write(pctrl->emmc_ioc, offset, index, 1,
					   value);
		return 0;
	case PIN_CONFIG_OUTPUT_IMPEDANCE_OHMS:
		if (argument != 40)
			return -EINVAL;

		value = QRK3588_EMMC_DS_40_OHM;
		if (!bank) {
			offset = EMMC_GPIO2A_DS_L;
		} else if (index < 4) {
			offset = EMMC_GPIO2D_DS_L;
		} else {
			offset = EMMC_GPIO2D_DS_H;
			index -= 4;
		}
		qrk3588_hiword_field_write(pctrl->emmc_ioc, offset, index * 4,
					   4, value);
		return 0;
	default:
		return -EOPNOTSUPP;
	}

	offset = bank ? EMMC_GPIO2D_PULL : EMMC_GPIO2A_PULL;
	qrk3588_hiword_field_write(pctrl->emmc_ioc, offset, index * 2, 2,
				   value);

	return 0;
}

static int qrk3588_pin_config_group_set(struct pinctrl_dev *pctldev,
					unsigned int selector,
					unsigned long *configs,
					unsigned int num_configs)
{
	const struct group_desc *group;
	struct qrk3588_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;
	unsigned int j;
	int ret;

	group = pinctrl_generic_get_group(pctldev, selector);
	if (!group)
		return -EINVAL;

	for (i = 0; i < group->grp.npins; i++) {
		for (j = 0; j < num_configs; j++) {
			ret = qrk3588_emmc_pin_config_set(pctrl,
							  group->grp.pins[i],
							  configs[j]);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct pinctrl_ops qrk3588_pinctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_group,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinmux_ops qrk3588_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = qrk3588_set_mux,
	.gpio_request_enable = qrk3588_gpio_request_enable,
	.gpio_disable_free = qrk3588_gpio_disable_free,
	.strict = true,
};

static const struct pinconf_ops qrk3588_pinconf_ops = {
	.is_generic = true,
	.pin_config_group_set = qrk3588_pin_config_group_set,
};

static const struct pinctrl_desc qrk3588_pinctrl_desc = {
	.name = "qrk3588-pinctrl",
	.pins = qrk3588_pins,
	.npins = ARRAY_SIZE(qrk3588_pins),
	.pctlops = &qrk3588_pinctrl_ops,
	.pmxops = &qrk3588_pinmux_ops,
	.confops = &qrk3588_pinconf_ops,
	.owner = THIS_MODULE,
};

static int qrk3588_register_groups(struct qrk3588_pinctrl *pctrl)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(qrk3588_groups); i++) {
		struct qrk3588_pin_group *group = &qrk3588_groups[i];

		ret = pinctrl_generic_add_group(pctrl->pctldev, group->name,
						group->pins, group->npins,
						group);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int qrk3588_register_functions(struct qrk3588_pinctrl *pctrl)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(qrk3588_functions); i++) {
		struct qrk3588_pin_function *function;

		function = &qrk3588_functions[i];
		ret = pinmux_generic_add_function(pctrl->pctldev, function->name,
						  function->groups,
						   function->ngroups,
						   function);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int qrk3588_pinctrl_probe(struct platform_device *pdev)
{
	struct qrk3588_pinctrl *pctrl;
	int ret;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->pmu1_ioc = devm_platform_ioremap_resource_byname(pdev, "pmu1-ioc");
	if (IS_ERR(pctrl->pmu1_ioc))
		return PTR_ERR(pctrl->pmu1_ioc);

	pctrl->pmu2_ioc = devm_platform_ioremap_resource_byname(pdev,
								"pmu2-ioc");
	if (IS_ERR(pctrl->pmu2_ioc))
		return PTR_ERR(pctrl->pmu2_ioc);

	pctrl->bus_ioc = devm_platform_ioremap_resource_byname(pdev,
							       "bus-ioc");
	if (IS_ERR(pctrl->bus_ioc))
		return PTR_ERR(pctrl->bus_ioc);

	pctrl->emmc_ioc = devm_platform_ioremap_resource_byname(pdev,
								"emmc-ioc");
	if (IS_ERR(pctrl->emmc_ioc))
		return PTR_ERR(pctrl->emmc_ioc);

	ret = devm_pinctrl_register_and_init(&pdev->dev, &qrk3588_pinctrl_desc,
					     pctrl, &pctrl->pctldev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register pin controller\n");

	ret = qrk3588_register_groups(pctrl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register pin groups\n");

	ret = qrk3588_register_functions(pctrl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register pin functions\n");

	return pinctrl_enable(pctrl->pctldev);
}

static const struct of_device_id qrk3588_pinctrl_of_match[] = {
	{ .compatible = "quest,qrk3588-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, qrk3588_pinctrl_of_match);

static struct platform_driver qrk3588_pinctrl_driver = {
	.probe = qrk3588_pinctrl_probe,
	.driver = {
		.name = "qrk3588-pinctrl",
		.of_match_table = qrk3588_pinctrl_of_match,
	},
};
module_platform_driver(qrk3588_pinctrl_driver);

MODULE_AUTHOR("ninozhang001@gmail.com");
MODULE_DESCRIPTION("Quest QRK3588 pin controller");
MODULE_LICENSE("GPL");
