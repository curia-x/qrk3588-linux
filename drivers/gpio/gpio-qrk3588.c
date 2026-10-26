// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 ninozhang001@gmail.com

#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define QRK3588_GPIO_PORT_COUNT	32
#define QRK3588_GPIO_BANK_COUNT	5

#define GPIO_SWPORT_DR_L	0x0000
#define GPIO_SWPORT_DDR_L	0x0008
#define GPIO_EXT_PORT		0x0070

struct qrk3588_gpio {
	void __iomem *base;
	struct gpio_chip chip;
	u32 bank_id;
};

static u32 qrk3588_gpio_hw_valid_mask(unsigned int bank)
{
	u32 mask = U32_MAX;

	switch (bank) {
	case 0:
		return mask & ~BIT(12) & ~BIT(31);
	case 1:
		return mask;
	case 2:
		return mask & ~BIT(4) & ~BIT(5) & ~BIT(23) & ~BIT(24);
	case 3:
		return mask & ~BIT(30) & ~BIT(31);
	case 4:
		return mask & ~BIT(23) & ~BIT(30) & ~BIT(31);
	default:
		return 0;
	}
}

static void __iomem *qrk3588_gpio_half_reg(struct qrk3588_gpio *gpio,
					   u32 low_offset,
					   unsigned int offset)
{
	return gpio->base + low_offset + (offset >= 16 ? 4 : 0);
}

static u32 qrk3588_gpio_half_bit(unsigned int offset)
{
	return BIT(offset & 0xf);
}

static void qrk3588_gpio_write_bit(struct qrk3588_gpio *gpio,
				   u32 low_offset, unsigned int offset,
				   bool value)
{
	void __iomem *reg = qrk3588_gpio_half_reg(gpio, low_offset, offset);
	u32 bit = qrk3588_gpio_half_bit(offset);
	u32 data = bit << 16;

	if (value)
		data |= bit;

	writel(data, reg);
}

static int qrk3588_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct qrk3588_gpio *gpio = gpiochip_get_data(chip);

	qrk3588_gpio_write_bit(gpio, GPIO_SWPORT_DDR_L, offset, false);
	return 0;
}

static int qrk3588_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct qrk3588_gpio *gpio = gpiochip_get_data(chip);

	/* Set the latch first so changing direction cannot produce a glitch. */
	qrk3588_gpio_write_bit(gpio, GPIO_SWPORT_DR_L, offset, value);
	qrk3588_gpio_write_bit(gpio, GPIO_SWPORT_DDR_L, offset, true);
	return 0;
}

static int qrk3588_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct qrk3588_gpio *gpio = gpiochip_get_data(chip);
	void __iomem *reg;

	reg = qrk3588_gpio_half_reg(gpio, GPIO_SWPORT_DDR_L, offset);
	return readl(reg) & qrk3588_gpio_half_bit(offset) ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int qrk3588_gpio_get_value(struct gpio_chip *chip,
				  unsigned int offset)
{
	struct qrk3588_gpio *gpio = gpiochip_get_data(chip);

	return !!(readl(gpio->base + GPIO_EXT_PORT) & BIT(offset));
}

static int qrk3588_gpio_set_value(struct gpio_chip *chip,
				  unsigned int offset, int value)
{
	struct qrk3588_gpio *gpio = gpiochip_get_data(chip);

	qrk3588_gpio_write_bit(gpio, GPIO_SWPORT_DR_L, offset, value);
	return 0;
}

static int qrk3588_gpio_init_valid_mask(struct gpio_chip *chip,
					unsigned long *valid_mask,
					unsigned int ngpios)
{
	struct qrk3588_gpio *gpio = gpiochip_get_data(chip);
	unsigned long hw_valid_mask[BITS_TO_LONGS(QRK3588_GPIO_PORT_COUNT)];
	u32 mask = qrk3588_gpio_hw_valid_mask(gpio->bank_id);

	bitmap_from_arr32(hw_valid_mask, &mask, QRK3588_GPIO_PORT_COUNT);
	bitmap_and(valid_mask, valid_mask, hw_valid_mask, ngpios);
	return 0;
}

static int qrk3588_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qrk3588_gpio *gpio;
	struct clk *pclk;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	ret = device_property_read_u32(dev, "quest,bank-id", &gpio->bank_id);
	if (ret)
		return dev_err_probe(dev, ret, "missing bank ID\n");
	if (gpio->bank_id >= QRK3588_GPIO_BANK_COUNT)
		return dev_err_probe(dev, -EINVAL, "invalid bank ID %u\n",
				     gpio->bank_id);

	pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk),
				     "failed to enable peripheral clock\n");

	gpio->chip.label = dev_name(dev);
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = QRK3588_GPIO_PORT_COUNT;
	gpio->chip.request = gpiochip_generic_request;
	gpio->chip.free = gpiochip_generic_free;
	gpio->chip.get_direction = qrk3588_gpio_get_direction;
	gpio->chip.direction_input = qrk3588_gpio_direction_input;
	gpio->chip.direction_output = qrk3588_gpio_direction_output;
	gpio->chip.get = qrk3588_gpio_get_value;
	gpio->chip.set = qrk3588_gpio_set_value;
	gpio->chip.set_config = gpiochip_generic_config;
	gpio->chip.init_valid_mask = qrk3588_gpio_init_valid_mask;

	return devm_gpiochip_add_data(dev, &gpio->chip, gpio);
}

static const struct of_device_id qrk3588_gpio_of_match[] = {
	{ .compatible = "quest,qrk3588-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, qrk3588_gpio_of_match);

static struct platform_driver qrk3588_gpio_driver = {
	.probe = qrk3588_gpio_probe,
	.driver = {
		.name = "qrk3588-gpio",
		.of_match_table = qrk3588_gpio_of_match,
	},
};
module_platform_driver(qrk3588_gpio_driver);

MODULE_AUTHOR("ninozhang001@gmail.com");
MODULE_DESCRIPTION("Quest QRK3588 GPIO controller");
MODULE_LICENSE("GPL");
