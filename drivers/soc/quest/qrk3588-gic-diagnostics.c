// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 ninozhang001@gmail.com */

#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#define QRK3588_GICT_BASE	0xfe620000
#define QRK3588_GICT_SIZE	SZ_64K
#define QRK3588_ITS0_BASE	0xfe640000
#define QRK3588_ITS1_BASE	0xfe660000
#define QRK3588_CRU_BASE	0xfd7c0000

#define CRU_GATE_CON32		0x0880
#define CRU_GATE_CON34		0x0888

#define GITS_CBASER		0x0080
#define GITS_CWRITER		0x0088
#define GITS_CREADR		0x0090

#define GICT_ERR_STATUS(n)	(0x0010 + (n) * 0x40)
#define GICT_ERR_ADDR(n)	(0x0018 + (n) * 0x40)
#define GICT_ERR_MISC0(n)	(0x0020 + (n) * 0x40)
#define GICT_ERR_MISC1(n)	(0x0028 + (n) * 0x40)
#define GICT_ERRGSR		0xe000
#define GICT_ERRIDR		0xffc8

#define GICT_ERR_STATUS_VALID	BIT_ULL(30)
#define GICT_ERRIDR_NUM_MASK	GENMASK(15, 0)
#define GICT_MAX_RECORDS	(GICT_ERRGSR / 0x40)

static void __init qrk3588_its_queue_diagnostics(phys_addr_t phys)
{
	void __iomem *base;

	base = ioremap(phys, SZ_4K);
	if (!base) {
		pr_err("QRK3588 ITS@%pa: cannot map registers\n", &phys);
		return;
	}

	pr_info("QRK3588 ITS@%pa: CBASER=%#llx CWRITER=%#llx CREADR=%#llx\n",
		&phys, readq(base + GITS_CBASER),
		readq(base + GITS_CWRITER), readq(base + GITS_CREADR));
	iounmap(base);
}

static int __init qrk3588_gic_diagnostics_init(void)
{
	void __iomem *base;
	u64 status;
	u32 records;
	u32 i;

	qrk3588_its_queue_diagnostics(QRK3588_ITS0_BASE);
	qrk3588_its_queue_diagnostics(QRK3588_ITS1_BASE);

	base = ioremap(QRK3588_GICT_BASE, QRK3588_GICT_SIZE);
	if (!base) {
		pr_err("QRK3588 GICT: cannot map %#x\n",
			 QRK3588_GICT_BASE);
		return -ENOMEM;
	}

	records = readl(base + GICT_ERRIDR) & GICT_ERRIDR_NUM_MASK;
	if (records > GICT_MAX_RECORDS) {
		pr_err("QRK3588 GICT: invalid record count %u\n", records);
		iounmap(base);
		return -EINVAL;
	}
	pr_info("QRK3588 GICT: records=%u groups=%#llx\n", records,
		readq(base + GICT_ERRGSR));

	for (i = 0; i < records; i++) {
		status = readq(base + GICT_ERR_STATUS(i));
		if (!(status & GICT_ERR_STATUS_VALID))
			continue;

		pr_err("QRK3588 GICT: record=%u status=%#llx addr=%#llx misc0=%#llx misc1=%#llx\n",
		       i, status, readq(base + GICT_ERR_ADDR(i)),
		       readq(base + GICT_ERR_MISC0(i)),
		       readq(base + GICT_ERR_MISC1(i)));
	}

	iounmap(base);
	return 0;
}
early_initcall(qrk3588_gic_diagnostics_init);

static int __init qrk3588_its_clock_diagnostics_late(void)
{
	void __iomem *base;

	base = ioremap(QRK3588_CRU_BASE, SZ_4K);
	if (!base) {
		pr_err("QRK3588 ITS clock: cannot map CRU\n");
		return -ENOMEM;
	}

	/* Read-only sample after the clock framework's unused-clock sweep. */
	pr_info("QRK3588 ITS clock late: gate32=%#x gate34=%#x\n",
		readl(base + CRU_GATE_CON32), readl(base + CRU_GATE_CON34));
	iounmap(base);
	return 0;
}
late_initcall_sync(qrk3588_its_clock_diagnostics_late);
