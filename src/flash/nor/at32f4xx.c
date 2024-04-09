// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

#define ERR_CHECK(p) do {			\
	int __ret = (p);			\
	if (__ret != ERROR_OK) return __ret;	\
} while (0)

#define min_t(type,x,y) \
    ({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })
#define max_t(type,x,y) \
    ({ type __x = (x); type __y = (y); __x > __y ? __x: __y; })

/* flash unlock keys */
#define KEY1				0x45670123
#define KEY2				0xCDEF89AB

/* flash erase timeout values */
#define FLASH_WRITE_TIMEOUT               100
#define FLASH_SECTOR_ERASE_TIMEOUT        1000
#define FLASH_MASS_ERASE_TIMEOUT          120000

#define BANK1_BASE_ADDR                   0x08000000
#define SPIM_BASE_ADDR                    0x08400000

/* Embedded Flash Controller register offsets. */
#define EFC_PSR             0x00
#define EFC_UNLOCK          0x04
#define EFC_USD_UNLOCK      0x08
#define EFC_STS             0x0C
#define EFC_CTRL            0x10
#define EFC_ADDR            0x14
#define EFC_USD             0x1C
#define EFC_EPPS            0x20
#define EFC_EPPS1           0x2C

/* flash ctrl register bits */
#define EFCCTRL_FPRGM			(1 << 0)
#define EFCCTRL_SECERS			(1 << 1)
#define EFCCTRL_BANKERS			(1 << 2)
#define EFCCTRL_USDPRGM			(1 << 4)
#define EFCCTRL_USDERS			(1 << 5)
#define EFCCTRL_ERSTR			(1 << 6)
#define EFCCTRL_OPLK			(1 << 7)
#define EFCCTRL_USDULKS			(1 << 9)

/* flash sts register bits */
#define EFCSTS_OBF			(1 << 0)
#define EFCSTS_PRGMERR			(1 << 2)
#define EFCSTS_EPPERR			(1 << 4)
#define EFCSTS_ODF			(1 << 5)

/* flash usd bits */
#define EFCUSD_USDERR			0
#define EFCUSD_FAP			1
#define EFCUSD_WDT_ATO_EN		2
#define EFCUSD_DEPSLP_RST		3
#define EFCUSD_STDBY_RST		4

struct at32_usd_data {
	uint8_t fap;
	uint8_t ssb;
	uint16_t data;
	uint32_t protection;
};

struct at32_spim_info {
	uint32_t is_spim;
	uint32_t io_mux;
	uint32_t flash_type;
	uint32_t flash_size;
	uint32_t sector_size;
};

struct at32_sub_bank {
	struct flash_bank *bank;
	uint32_t reg_base;
	uint32_t size;
	uint32_t num_sectors;
	uint32_t base;
};

struct at32_flash_info {
	uint32_t pid;
	uint32_t flash_size;
	uint16_t sector_size;
	uint32_t bank_addr;
	struct at32_sub_bank sub_bank[2];
	uint8_t probed;
	uint32_t usd_addr;
	const struct artery_chip *chip;
	struct at32_usd_data usd_data;
	struct at32_spim_info spim_info;
};

#define AT32_PRODUCT_ID_ADDR              0xE0042000

static int at32x_mass_erase(struct flash_bank *bank);
static int at32x_get_product_id(struct flash_bank *bank, uint32_t *product_id);
static int at32x_write_block(struct at32_sub_bank *sub_bank,
			     const uint8_t *buffer,
			     uint32_t address, uint32_t count);
static int at32x_sub_bank_mass_erase(struct at32_sub_bank *sub_bank);

struct mcu_type_info {
	uint32_t flash_reg;
	uint32_t usd_addr;
	const char *name;
};

struct mcu_type_info at32f403 = {
	0x40022000, 0x1FFFF800, "AT32F403"
};
struct mcu_type_info at32f413 = {
	0x40022000, 0x1FFFF800, "AT32F413"
};
struct mcu_type_info at32f415 = {
	0x40022000, 0x1FFFF800, "AT32F415"
};
struct mcu_type_info at32f403a = {
	0x40022000, 0x1FFFF800, "AT32F403A"
};
struct mcu_type_info at32f407 = {
	0x40022000, 0x1FFFF800, "AT32F407"
};
struct mcu_type_info at32f421 = {
	0x40022000, 0x1FFFF800, "AT32F421"
};
struct mcu_type_info at32f435 = {
	0x40023C00, 0x1FFFC000, "AT32F435"
};
struct mcu_type_info at32f437 = {
	0x40023C00, 0x1FFFC000, "AT32F437"
};
struct mcu_type_info at32f425 = {
	0x40022000, 0x1FFFF800, "AT32F425"
};
struct mcu_type_info at32l021 = {
	0x40022000, 0x1FFFF800, "AT32L021"
};
struct mcu_type_info at32wb415 = {
	0x40022000, 0x1FFFF800, "AT32WB415"
};
struct mcu_type_info at32f423 = {
	0x40023C00, 0x1FFFF800, "AT32F423"
};

struct artery_chip {
	uint32_t pid;
	uint32_t flash_size_kb;
	uint32_t sector_size;
	const struct mcu_type_info *type;
	const char *suffix;
};

static const struct artery_chip known_artery_chips[] = {
{ 0x70050242	, 256	, 2048	, &at32f403a	, "CCT7"	},
{ 0x70050243	, 256	, 2048	, &at32f403a	, "CCU7"	},
{ 0x700502CF	, 512	, 2048	, &at32f403a	, "CET7"	},
{ 0x700502D0	, 512	, 2048	, &at32f403a	, "CEU7"	},
{ 0x70050346	, 1024	, 2048	, &at32f403a	, "CGT7"	},
{ 0x70050347	, 1024	, 2048	, &at32f403a	, "CGU7"	},
{ 0x70050241	, 256	, 2048	, &at32f403a	, "RCT7"	},
{ 0x700502CE	, 512	, 2048	, &at32f403a	, "RET7"	},
{ 0x70050345	, 1024	, 2048	, &at32f403a	, "RGT7"	},
{ 0x70050240	, 256	, 2048	, &at32f403a	, "VCT7"	},
{ 0x700502CD	, 512	, 2048	, &at32f403a	, "VET7"	},
{ 0x70050344	, 1024	, 2048	, &at32f403a	, "VGT7"	},
{ 0xF0050355	, 1024	, 2048	, &at32f403a	, "VGW"	},
{ 0x700301CF	, 128	, 1024	, &at32f403	, "CBT6"	},
{ 0x70050243	, 256	, 2048	, &at32f403	, "CCT6"	},
{ 0x7005024E	, 256	, 2048	, &at32f403	, "CCU6"	},
{ 0x700502CB	, 512	, 2048	, &at32f403	, "CET6"	},
{ 0x700502CD	, 512	, 2048	, &at32f403	, "CEU6"	},
{ 0x70050347	, 1024	, 2048	, &at32f403	, "CGT6"	},
{ 0x7005034C	, 1024	, 2048	, &at32f403	, "CGU6"	},
{ 0x70050242	, 256	, 2048	, &at32f403	, "RCT6"	},
{ 0x700502CA	, 512	, 2048	, &at32f403	, "RET6"	},
{ 0x70050346	, 1024	, 2048	, &at32f403	, "RGT6"	},
{ 0x70050241	, 256	, 2048	, &at32f403	, "VCT6"	},
{ 0x700502C9	, 512	, 2048	, &at32f403	, "VET6"	},
{ 0x70050345	, 1024	, 2048	, &at32f403	, "VGT6"	},
{ 0x70050240	, 256	, 2048	, &at32f403	, "ZCT6"	},
{ 0x700502C8	, 512	, 2048	, &at32f403	, "ZET6"	},
{ 0x70050344	, 1024	, 2048	, &at32f403	, "ZGT6"	},
{ 0x70050254	, 256	, 2048	, &at32f407	, "AVCT7"	},
{ 0x70050353	, 1024	, 2048	, &at32f407	, "AVGT7"	},
{ 0x7005024A	, 256	, 2048	, &at32f407	, "RCT7"	},
{ 0x700502D2	, 512	, 2048	, &at32f407	, "RET7"	},
{ 0x7005034C	, 1024	, 2048	, &at32f407	, "RGT7"	},
{ 0x70050249	, 256	, 2048	, &at32f407	, "VCT7"	},
{ 0x700502D1	, 512	, 2048	, &at32f407	, "VET7"	},
{ 0x7005034B	, 1024	, 2048	, &at32f407	, "VGT7"	},
{ 0x70030106	, 64	, 1024	, &at32f413	, "C8T7"	},
{ 0x700301C3	, 128	, 1024	, &at32f413	, "CBT7"	},
{ 0x700301CA	, 128	, 1024	, &at32f413	, "CBU7"	},
{ 0x70030242	, 256	, 2048	, &at32f413	, "CCT7"	},
{ 0x70030247	, 256	, 2048	, &at32f413	, "CCU7"	},
{ 0x700301C5	, 128	, 1024	, &at32f413	, "KBU7-4"	},
{ 0x70030244	, 256	, 2048	, &at32f413	, "KCU7-4"	},
{ 0x700301C1	, 128	, 1024	, &at32f413	, "RBT7"	},
{ 0x70030240	, 256	, 2048	, &at32f413	, "RCT7"	},
{ 0x700301CB	, 128	, 1024	, &at32f413	, "TBU7"	},
{ 0x70030109	, 64	, 1024	, &at32f415	, "C8T7"	},
{ 0x700301C5	, 128	, 1024	, &at32f415	, "CBT7"	},
{ 0x700301CD	, 128	, 1024	, &at32f415	, "CBU7"	},
{ 0x70030241	, 256	, 2048	, &at32f415	, "CCT7"	},
{ 0x7003024C	, 256	, 2048	, &at32f415	, "CCU7"	},
{ 0x7003010A	, 64	, 1024	, &at32f415	, "K8U7-4"	},
{ 0x700301C6	, 128	, 1024	, &at32f415	, "KBU7-4"	},
{ 0x70030242	, 256	, 2048	, &at32f415	, "KCU7-4"	},
{ 0x7003010B	, 64	, 1024	, &at32f415	, "R8T7-7"	},
{ 0x70030108	, 64	, 1024	, &at32f415	, "R8T7"	},
{ 0x700301C7	, 128	, 1024	, &at32f415	, "RBT7-7"	},
{ 0x700301C4	, 128	, 1024	, &at32f415	, "RBT7"	},
{ 0x700301CF	, 128	, 1024	, &at32f415	, "RBW"	},
{ 0x70030243	, 256	, 2048	, &at32f415	, "RCT7-7"	},
{ 0x70030240	, 256	, 2048	, &at32f415	, "RCT7"	},
{ 0x7003024E	, 256	, 2048	, &at32f415	, "RCW"	},
{ 0x5001000C	, 16	, 1024	, &at32f421	, "C4T7"	},
{ 0x50020086	, 32	, 1024	, &at32f421	, "C6T7"	},
{ 0x50020100	, 64	, 1024	, &at32f421	, "C8T7"	},
{ 0xD0020100	, 64	, 1024	, &at32f421	, "C8W-YY"	},
{ 0x50020117	, 64	, 1024	, &at32f421	, "C8W"	},
{ 0x50010011	, 16	, 1024	, &at32f421	, "F4P7"	},
{ 0x50010010	, 16	, 1024	, &at32f421	, "F4U7"	},
{ 0x5002008B	, 32	, 1024	, &at32f421	, "F6P7"	},
{ 0x5002008A	, 32	, 1024	, &at32f421	, "F6U7"	},
{ 0x50020105	, 64	, 1024	, &at32f421	, "F8P7"	},
{ 0x50020104	, 64	, 1024	, &at32f421	, "F8U7"	},
{ 0x50010014	, 16	, 1024	, &at32f421	, "G4U7"	},
{ 0x50020093	, 32	, 1024	, &at32f421	, "G6U7"	},
{ 0x50020112	, 64	, 1024	, &at32f421	, "G8U7"	},
{ 0x5001000D	, 16	, 1024	, &at32f421	, "K4T7"	},
{ 0x5001000F	, 16	, 1024	, &at32f421	, "K4U7-4"	},
{ 0x5001000E	, 16	, 1024	, &at32f421	, "K4U7"	},
{ 0x50020087	, 32	, 1024	, &at32f421	, "K6T7"	},
{ 0x50020089	, 32	, 1024	, &at32f421	, "K6U7-4"	},
{ 0x50020088	, 32	, 1024	, &at32f421	, "K6U7"	},
{ 0x50020101	, 64	, 1024	, &at32f421	, "K8T7"	},
{ 0x50020103	, 64	, 1024	, &at32f421	, "K8U7-4"	},
{ 0x50020102	, 64	, 1024	, &at32f421	, "K8U7"	},
{ 0x50010016	, 16	, 1024	, &at32f421	, "PF4P7"	},
{ 0x50020115	, 64	, 1024	, &at32f421	, "PF8P7"	},
{ 0x7003210B	, 64	, 1024	, &at32f423	, "C8T7"	},
{ 0x7003210E	, 64	, 1024	, &at32f423	, "C8U7"	},
{ 0x700A21CA	, 128	, 1024	, &at32f423	, "CBT7"	},
{ 0x700A21CD	, 128	, 1024	, &at32f423	, "CBU7"	},
{ 0x700A3249	, 256	, 2048	, &at32f423	, "CCT7"	},
{ 0x700A324C	, 256	, 2048	, &at32f423	, "CCU7"	},
{ 0x70032115	, 64	, 1024	, &at32f423	, "K8U7-4"	},
{ 0x700A21D4	, 128	, 1024	, &at32f423	, "KBU7-4"	},
{ 0x700A3253	, 256	, 2048	, &at32f423	, "KCU7-4"	},
{ 0x70032108	, 64	, 1024	, &at32f423	, "R8T7-7"	},
{ 0x70032105	, 64	, 1024	, &at32f423	, "R8T7"	},
{ 0x700A21C7	, 128	, 1024	, &at32f423	, "RBT7-7"	},
{ 0x700A21C4	, 128	, 1024	, &at32f423	, "RBT7"	},
{ 0x700A3246	, 256	, 2048	, &at32f423	, "RCT7-7"	},
{ 0x700A3243	, 256	, 2048	, &at32f423	, "RCT7"	},
{ 0x70032112	, 64	, 1024	, &at32f423	, "T8U7"	},
{ 0x700A21D1	, 128	, 1024	, &at32f423	, "TBU7"	},
{ 0x700A3250	, 256	, 2048	, &at32f423	, "TCU7"	},
{ 0x70032102	, 64	, 1024	, &at32f423	, "V8T7"	},
{ 0x700A21C1	, 128	, 1024	, &at32f423	, "VBT7"	},
{ 0x700A3240	, 256	, 2048	, &at32f423	, "VCT7"	},
{ 0x50092087	, 32	, 1024	, &at32f425	, "C6T7"	},
{ 0x5009208A	, 32	, 1024	, &at32f425	, "C6U7"	},
{ 0x50092106	, 64	, 1024	, &at32f425	, "C8T7"	},
{ 0x50092109	, 64	, 1024	, &at32f425	, "C8U7"	},
{ 0x50092093	, 32	, 1024	, &at32f425	, "F6P7"	},
{ 0x50092112	, 64	, 1024	, &at32f425	, "F8P7"	},
{ 0x50092096	, 32	, 1024	, &at32f425	, "G6U7"	},
{ 0x50092115	, 64	, 1024	, &at32f425	, "G8U7"	},
{ 0x5009208D	, 32	, 1024	, &at32f425	, "K6T7"	},
{ 0x50092090	, 32	, 1024	, &at32f425	, "K6U7-4"	},
{ 0x5009210C	, 64	, 1024	, &at32f425	, "K8T7"	},
{ 0x5009210F	, 64	, 1024	, &at32f425	, "K8U7-4"	},
{ 0x50092084	, 32	, 1024	, &at32f425	, "R6T7-7"	},
{ 0x50092081	, 32	, 1024	, &at32f425	, "R6T7"	},
{ 0x50092103	, 64	, 1024	, &at32f425	, "R8T7-7"	},
{ 0x50092100	, 64	, 1024	, &at32f425	, "R8T7"	},
{ 0x7008449A	, 192	, 4096	, &at32f435	, "CCT7-W"	},
{ 0x7008324B	, 256	, 2048	, &at32f435	, "CCT7"	},
{ 0x7008449D	, 192	, 4096	, &at32f435	, "CCU7-W"	},
{ 0x7008324E	, 256	, 2048	, &at32f435	, "CCU7"	},
{ 0x700844D9	, 960	, 4096	, &at32f435	, "CGT7-W"	},
{ 0x7008334A	, 1024	, 2048	, &at32f435	, "CGT7"	},
{ 0x700844DC	, 960	, 4096	, &at32f435	, "CGU7-W"	},
{ 0x7008334D	, 1024	, 2048	, &at32f435	, "CGU7"	},
{ 0x70084558	, 4032	, 4096	, &at32f435	, "CMT7-E"	},
{ 0x70084549	, 4032	, 4096	, &at32f435	, "CMT7"	},
{ 0x7008455B	, 4032	, 4096	, &at32f435	, "CMU7-E"	},
{ 0x7008454C	, 4032	, 4096	, &at32f435	, "CMU7"	},
{ 0x70083248	, 256	, 2048	, &at32f435	, "RCT7"	},
{ 0x70083347	, 1024	, 2048	, &at32f435	, "RGT7"	},
{ 0x70084546	, 4032	, 4096	, &at32f435	, "RMT7"	},
{ 0x70083245	, 256	, 2048	, &at32f435	, "VCT7"	},
{ 0x70083344	, 1024	, 2048	, &at32f435	, "VGT7"	},
{ 0x70084543	, 4032	, 4096	, &at32f435	, "VMT7"	},
{ 0x70083242	, 256	, 2048	, &at32f435	, "ZCT7"	},
{ 0x70083341	, 1024	, 2048	, &at32f435	, "ZGT7"	},
{ 0x70084540	, 4032	, 4096	, &at32f435	, "ZMT7"	},
{ 0x70083257	, 256	, 2048	, &at32f437	, "RCT7"	},
{ 0x70083356	, 1024	, 2048	, &at32f437	, "RGT7"	},
{ 0x70084555	, 4032	, 4096	, &at32f437	, "RMT7"	},
{ 0x70083254	, 256	, 2048	, &at32f437	, "VCT7"	},
{ 0x70083353	, 1024	, 2048	, &at32f437	, "VGT7"	},
{ 0x70084552	, 4032	, 4096	, &at32f437	, "VMT7"	},
{ 0x70083251	, 256	, 2048	, &at32f437	, "ZCT7"	},
{ 0x70083350	, 1024	, 2048	, &at32f437	, "ZGT7"	},
{ 0x7008454F	, 4032	, 4096	, &at32f437	, "ZMT7"	},
{ 0x10012006	, 16	, 1024	, &at32l021	, "C4T7"	},
{ 0x1001208D	, 32	, 1024	, &at32l021	, "C6T7"	},
{ 0x10012114	, 64	, 1024	, &at32l021	, "C8T7"	},
{ 0x10012001	, 16	, 1024	, &at32l021	, "F4P7"	},
{ 0x10012002	, 16	, 1024	, &at32l021	, "F4U7"	},
{ 0x10012088	, 32	, 1024	, &at32l021	, "F6P7"	},
{ 0x10012089	, 32	, 1024	, &at32l021	, "F6U7"	},
{ 0x1001210F	, 64	, 1024	, &at32l021	, "F8P7"	},
{ 0x10012110	, 64	, 1024	, &at32l021	, "F8U7"	},
{ 0x10012000	, 16	, 1024	, &at32l021	, "G4U7"	},
{ 0x10012087	, 32	, 1024	, &at32l021	, "G6U7"	},
{ 0x1001210E	, 64	, 1024	, &at32l021	, "G8U7"	},
{ 0x10012005	, 16	, 1024	, &at32l021	, "K4T7"	},
{ 0x10012003	, 16	, 1024	, &at32l021	, "K4U7-4"	},
{ 0x10012004	, 16	, 1024	, &at32l021	, "K4U7"	},
{ 0x1001208C	, 32	, 1024	, &at32l021	, "K6T7"	},
{ 0x1001208A	, 32	, 1024	, &at32l021	, "K6U7-4"	},
{ 0x1001208B	, 32	, 1024	, &at32l021	, "K6U7"	},
{ 0x10012113	, 64	, 1024	, &at32l021	, "K8T7"	},
{ 0x10012111	, 64	, 1024	, &at32l021	, "K8U7-4"	},
{ 0x10012112	, 64	, 1024	, &at32l021	, "K8U7"	},
{ 0x70030250	, 256	, 2048	, &at32wb415	, "CCU7-7"	},
{ 0 }
};

static int artery_find_chip_from_id(uint32_t pid, const struct artery_chip **pchip)
{
	const struct artery_chip *chip;
	for (chip = known_artery_chips; chip->pid; chip++) {
		if (chip->pid == pid) {
			if (pchip)
				*pchip = chip;
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

static int at32x_get_product_id(struct flash_bank *bank, uint32_t *product_id)
{
	struct target *target = bank->target;
	ERR_CHECK(target_read_u32(target, AT32_PRODUCT_ID_ADDR, product_id));
	return ERROR_OK;
}

static int at32x_init_spim(struct flash_bank *bank)
{
	uint32_t read_val;
	struct at32_flash_info *at32x_info = bank->driver_priv;
	const struct artery_chip *chip = at32x_info->chip;

	at32x_info->spim_info.sector_size = 4096;
	at32x_info->sector_size = at32x_info->spim_info.sector_size;
	at32x_info->flash_size = at32x_info->spim_info.flash_size;
	at32x_info->sub_bank[0].size = at32x_info->spim_info.flash_size;
	at32x_info->sub_bank[0].reg_base = chip->type->flash_reg + 0x80;
	
	/* enable gpio clock */
	ERR_CHECK(target_write_u32(bank->target, 0x40021018, 0xD));

	/* read gpioa pa8 config */
	ERR_CHECK(target_read_u32(bank->target, 0x40010804, &read_val));
	read_val &= ~0xf;
	read_val |= 0x9;
		
	ERR_CHECK(target_write_u32(bank->target, 0x40010804, read_val));

	/* read gpiob pb1, pb6, pb7 config */
	ERR_CHECK(target_read_u32(bank->target, 0x40010c00, &read_val));
	read_val &= ~0xff0000f0;
	read_val |= 0x99000090;
		
	ERR_CHECK(target_write_u32(bank->target, 0x40010c00, read_val));

	if (at32x_info->spim_info.io_mux) {

		/*read gpiob pb10, pb11  config*/
		ERR_CHECK(target_read_u32(bank->target, 0x40010c04, &read_val));
		read_val &= ~0x0000ff00;
		read_val |= 0x00009900;
		ERR_CHECK(target_write_u32(bank->target, 0x40010c04, read_val));

	} else {

		/*read gpiob pa11, pa12  config*/
		ERR_CHECK(target_read_u32(bank->target, 0x40010804, &read_val));
		read_val &= ~0x000ff000;
		read_val |= 0x00099000;
		ERR_CHECK(target_write_u32(bank->target, 0x40010804, read_val));
	}

	/*enable spif*/
	if (at32x_info->chip->type == &at32f403) {
		ERR_CHECK(target_write_u32(bank->target, 0x4001001c, 1 << 21));

	} else {
		ERR_CHECK(target_write_u32(bank->target, 0x40010030, 0x00000009));
	}
			
	/*flash type select*/
	ERR_CHECK(target_write_u32(bank->target, 0x40022088, at32x_info->spim_info.flash_type));

	LOG_INFO("%s%s spim flash size: 0x%" PRIx32 ", sector size: 0x%x",
		 chip->type->name, chip->suffix,
		 at32x_info->flash_size,
		 at32x_info->sector_size);

	return ERROR_OK;
}

static int at32x_init_main_flash(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	const struct artery_chip *chip = at32x_info->chip;

	if (at32x_info->bank_addr != BANK1_BASE_ADDR) {
		LOG_ERROR("Invalid flash bank address:0x %" PRIx32,
			  at32x_info->bank_addr);
		return ERROR_FAIL;
	}

	at32x_info->flash_size = chip->flash_size_kb << 10;
	at32x_info->sector_size = chip->sector_size;
	at32x_info->usd_addr = chip->type->usd_addr;
	at32x_info->sub_bank[0].reg_base = chip->type->flash_reg;
	at32x_info->sub_bank[0].size =
		min_t(uint32_t, at32x_info->flash_size,
		      (chip->flash_size_kb > 1024) ? 2 << 20 : 512 << 10);
	at32x_info->sub_bank[1].reg_base = chip->type->flash_reg + 0x40;
	at32x_info->sub_bank[1].size = at32x_info->flash_size
		- at32x_info->sub_bank[0].size;

	LOG_INFO("%s%s: main flash size: %" PRIu32 "kB, "
		 "sector size: %" PRIu32,
		 chip->type->name, chip->suffix,
		 at32x_info->flash_size >> 10,
		 at32x_info->sector_size);

	return ERROR_OK;
}

static int at32_get_device_info(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	const struct artery_chip *chip;
	unsigned int i;
	uint32_t base;

	ERR_CHECK(at32x_get_product_id(bank, &at32x_info->pid));

	ERR_CHECK(artery_find_chip_from_id(at32x_info->pid, &chip));
	at32x_info->chip = chip;
	
	if (at32x_info->spim_info.is_spim) {
		ERR_CHECK(at32x_init_spim(bank));
	} else {
		ERR_CHECK(at32x_init_main_flash(bank));
	}

	base = bank->base;
	for (i = 0; i < ARRAY_SIZE(at32x_info->sub_bank); i++) {
		struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[i];
		sub_bank->base = base;
		sub_bank->bank = bank;
		sub_bank->num_sectors = sub_bank->size
			/ at32x_info->sector_size;
		if (sub_bank->size == 0)
			continue;
		base += sub_bank->size;
		LOG_INFO(" ... sub-bank[%d] size: %" PRIu32 "kB",
			 i, at32x_info->sub_bank[i].size >> 10);
	}

	return ERROR_OK;
}

/* flash bank at32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(at32x_flash_bank_command)
{
	struct at32_flash_info *at32x_info;
	
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;
	at32x_info = malloc(sizeof(struct at32_flash_info));
	memset(at32x_info, 0, sizeof(*at32x_info));

	if (bank->base == SPIM_BASE_ADDR) {
		uint32_t io_mux, type, size;
		
		if (CMD_ARGC < 9)
			return ERROR_COMMAND_SYNTAX_ERROR;
		
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], io_mux);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], type);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], size);
		at32x_info->spim_info.is_spim = true;
		at32x_info->spim_info.io_mux = io_mux;
		at32x_info->spim_info.flash_type = type;
		at32x_info->spim_info.flash_size = size;

		LOG_INFO("spim flash io_mux: 0x%" PRIx32 ", type: 0x%" PRIx32 ", size: 0x%" PRIx32 "", io_mux, type, size);
	}
	bank->driver_priv = at32x_info;
	at32x_info->bank_addr = bank->base;
	return ERROR_OK;
}

static inline int at32x_get_flash_reg(
	struct at32_sub_bank *sub_bank, uint32_t reg)
{
	return (reg + sub_bank->reg_base);
}

static int sub_bank_write_reg(
	struct at32_sub_bank *sub_bank, uint32_t off, uint32_t value)
{
	return target_write_u32(sub_bank->bank->target,
				at32x_get_flash_reg(sub_bank, off),
				value);
}

static int sub_bank_read_reg(
	struct at32_sub_bank *sub_bank, uint32_t off, uint32_t *pvalue)
{
	return target_read_u32(sub_bank->bank->target,
			       at32x_get_flash_reg(sub_bank, off),
			       pvalue);
}

static int at32x_flash_unlock(struct at32_sub_bank *sub_bank)
{
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_UNLOCK, KEY1));
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_UNLOCK, KEY2));
	return ERROR_OK;
}

static int at32x_usd_unlock(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[0];
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_USD_UNLOCK, KEY1));
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_USD_UNLOCK, KEY2));
	return ERROR_OK;
}

static int at32x_wait_status_busy(struct at32_sub_bank *sub_bank, int timeout)
{
	struct at32_flash_info *at32x_info = sub_bank->bank->driver_priv;
	uint32_t status;
	int retval = ERROR_OK;
	
	/* Wait for busy to clear. */
	for (;;) {
		ERR_CHECK(sub_bank_read_reg(sub_bank, EFC_STS, &status));
		LOG_DEBUG(">status: 0x%" PRIx32 "", status);
		if (!(status & EFCSTS_OBF))
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	/* Log & clear errors. */
	if (status & (EFCSTS_EPPERR | EFCSTS_PRGMERR)) {
		LOG_ERROR("%s%s device programming failed",
			  at32x_info->chip->type->name,
			  at32x_info->chip->suffix);
		retval = ERROR_FAIL;
		sub_bank_write_reg(sub_bank, EFC_STS,
				   EFCSTS_EPPERR | EFCSTS_PRGMERR);
	}

	return retval;
}

static int at32x_read_usd_data(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t usd_data;

	/* read user and read protection option bytes */
	ERR_CHECK(target_read_u32(target, at32x_info->usd_addr, &usd_data));

	at32x_info->usd_data.fap = usd_data & 0xFF;
	at32x_info->usd_data.ssb = (usd_data >> 16) & 0xFF;

	/* read user data option bytes */
	ERR_CHECK(target_read_u32(target, at32x_info->usd_addr+4, &usd_data));

	at32x_info->usd_data.data = ((usd_data >> 8) & 0xFF00) | (usd_data & 0xFF);

	/* read write protection option bytes */
	ERR_CHECK(target_read_u32(target, at32x_info->usd_addr+8, &usd_data));

	at32x_info->usd_data.protection = ((usd_data >> 8) & 0xFF00) | (usd_data & 0xFF);

	ERR_CHECK(target_read_u32(target, at32x_info->usd_addr+0xc, &usd_data));

	at32x_info->usd_data.protection |= (((usd_data >> 8) & 0xFF00) | (usd_data & 0xFF)) << 16;

	return ERROR_OK;
}

static int at32x_erase_usd_data(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[0];
	uint32_t op = EFCCTRL_USDERS | EFCCTRL_USDULKS;

	at32x_read_usd_data(bank);

	ERR_CHECK(at32x_flash_unlock(sub_bank));

	ERR_CHECK(at32x_usd_unlock(bank));

	/* erase user system data */
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL, op));
	
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL,
				     op | EFCCTRL_ERSTR));

	ERR_CHECK(at32x_wait_status_busy(sub_bank, FLASH_SECTOR_ERASE_TIMEOUT));
	
	return ERROR_OK;
}

static int at32x_write_usd_data(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[0];
	struct target *target = bank->target;
	uint8_t usd_data[16];
	int retval;

	ERR_CHECK(at32x_flash_unlock(sub_bank));

	ERR_CHECK(at32x_usd_unlock(bank));

	/* program option bytes */
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL, 
				     EFCCTRL_USDPRGM | EFCCTRL_USDULKS));

	target_buffer_set_u16(target, usd_data, at32x_info->usd_data.fap);
	target_buffer_set_u16(target, usd_data + 2, at32x_info->usd_data.ssb);
	target_buffer_set_u16(target, usd_data + 4, at32x_info->usd_data.data & 0xff);
	target_buffer_set_u16(target, usd_data + 6, (at32x_info->usd_data.data >> 8) & 0xff);
	target_buffer_set_u16(target, usd_data + 8, at32x_info->usd_data.protection & 0xff);
	target_buffer_set_u16(target, usd_data + 10, (at32x_info->usd_data.protection >> 8) & 0xff);
	target_buffer_set_u16(target, usd_data + 12, (at32x_info->usd_data.protection >> 16) & 0xff);
	target_buffer_set_u16(target, usd_data + 14, (at32x_info->usd_data.protection >> 24) & 0xff);

	retval = at32x_write_block(sub_bank, usd_data, at32x_info->usd_addr, sizeof(usd_data) / 2);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			LOG_ERROR("working area required to erase options bytes");
		return retval;
	}

	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL, EFCCTRL_OPLK));

	return ERROR_OK;
}

static int at32x_protect_check(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[0];
	uint32_t protection;

	ERR_CHECK(sub_bank_read_reg(sub_bank, EFC_EPPS, &protection));

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

	return ERROR_OK;
}

static int at32x_sub_bank_erase(struct at32_sub_bank *sub_bank, unsigned int first, unsigned int last)
{
	struct flash_bank *bank = sub_bank->bank;
	unsigned int i, secsz = sub_bank->size / sub_bank->num_sectors;

	if ((first == 0) && (last == (sub_bank->num_sectors - 1)))
		return at32x_sub_bank_mass_erase(sub_bank);

	ERR_CHECK(at32x_flash_unlock(sub_bank));

	for (i = first; i <= last; i++) {
		ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL,
					     EFCCTRL_SECERS));
		ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_ADDR,
					     sub_bank->base + i*secsz));
		ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL,
					     EFCCTRL_SECERS | EFCCTRL_ERSTR));

		ERR_CHECK(at32x_wait_status_busy(sub_bank, FLASH_SECTOR_ERASE_TIMEOUT));

		bank->sectors[i].is_erased = 1;
	}

	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL, EFCCTRL_OPLK));

	return ERROR_OK;
}

static int at32x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	unsigned int i;

	LOG_INFO("Erase first sector = 0x%" PRIx32 ", last sector = 0x%"
		 PRIx32 " ", first, last);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return at32x_mass_erase(bank);

	for (i = 0; i < ARRAY_SIZE(at32x_info->sub_bank); i++) {
		struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[i];
		if (first < sub_bank->num_sectors) {
			unsigned int l = min_t(unsigned int, last,
					       sub_bank->num_sectors-1);
			ERR_CHECK(at32x_sub_bank_erase(sub_bank, first, l));
			if (last == l)
				break;
			first = 0;
		} else {
			first -= sub_bank->num_sectors;
		}
		last -= sub_bank->num_sectors;
	}

	return ERROR_OK;
}

static int at32x_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct at32_flash_info *at32x_info = bank->driver_priv;
	int retval;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = at32x_erase_usd_data(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s%s failed to erase options", at32x_info->chip->type->name, at32x_info->chip->suffix);
		return retval;
	}

	for (unsigned int i = first; i <= last; i++) {
		if (set)
			at32x_info->usd_data.protection &= ~(1 << i);
		else
			at32x_info->usd_data.protection |= (1 << i);
	}

	return at32x_write_usd_data(bank);
}

static int at32x_write_block(struct at32_sub_bank *sub_bank,
			     const uint8_t *buffer,
			     uint32_t address, uint32_t count)
{
	struct flash_bank *bank = sub_bank->bank;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t at32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32f1x.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(at32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(at32x_flash_write_code), at32x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, sub_bank->reg_base);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) & EFCSTS_PRGMERR) {
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			sub_bank_write_reg(sub_bank, EFC_STS, EFCSTS_PRGMERR);
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & EFCSTS_EPPERR) {
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			sub_bank_write_reg(sub_bank, EFC_STS, EFCSTS_EPPERR);
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int at32x_sub_bank_write(struct at32_sub_bank *sub_bank,
				const uint8_t *buffer,
				uint32_t offset, uint32_t count)
{
	struct target *target = sub_bank->bank->target;
	uint32_t words_remaining = count / 2;
	int retval, retval2;

	retval = at32x_flash_unlock(sub_bank);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	retval = sub_bank_write_reg(sub_bank, EFC_CTRL, EFCCTRL_FPRGM);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* try using a block write */
	retval = at32x_write_block(sub_bank, buffer, sub_bank->base + offset, words_remaining);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (words_remaining > 0) {
			uint16_t value;
			memcpy(&value, buffer, sizeof(uint16_t));

			retval = target_write_u16(target, sub_bank->base + offset, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;
			retval = at32x_wait_status_busy(sub_bank, 5);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;
			words_remaining--;
			buffer += 2;
			offset += 2;
		}
	}

reset_pg_and_lock:
	retval2 = sub_bank_write_reg(sub_bank, EFC_CTRL, EFCCTRL_OPLK);
	if (retval == ERROR_OK)
		retval = retval2;

	return retval;
}

static int at32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	uint8_t *new_buffer = NULL;
	int retval = ERROR_OK;
	unsigned int i;

	LOG_INFO("Write address = 0x%" PRIx32 ", count: 0x%" PRIx32 "",
		 (uint32_t)bank->base + offset,  count);
	
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required "
			  "2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. */
	if (count & 1) {
		new_buffer = malloc(count + 1);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no "
				  "memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		new_buffer[count++] = 0xff;
	}

	for (i = 0; i < ARRAY_SIZE(at32x_info->sub_bank); i++) {
		struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[i];
		if (offset < sub_bank->size) {
			uint32_t c = min_t(uint32_t, count, sub_bank->size);
			retval = at32x_sub_bank_write(sub_bank, buffer,
						      offset, c);
			if ((retval != ERROR_OK) || (count == c))
				break;
			offset = 0;
		} else {
			offset -= sub_bank->size;
		}
		count -= sub_bank->size;
	}

	if (new_buffer)
		free(new_buffer);

	return retval;
}


static int at32x_probe(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	int num_pages, num_prot_blocks, secsz;

	if (at32x_info->probed)
		return ERROR_OK;

	ERR_CHECK(at32_get_device_info(bank));

	secsz = at32x_info->sector_size;
	num_pages = at32x_info->flash_size / secsz;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	if (bank->prot_blocks) {
		free(bank->prot_blocks);
		bank->prot_blocks = NULL;
	}
	bank->size = at32x_info->flash_size;
	bank->num_sectors = num_pages;
	
	bank->sectors = alloc_block_array(0, secsz, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	num_prot_blocks = (at32x_info->flash_size + 4095) / 4096;
	if (num_prot_blocks > 32)
		num_prot_blocks = 32;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, 4096, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	if (num_prot_blocks == 32)
		bank->prot_blocks[31].size = (num_pages - (31 * 2)) * secsz;

	at32x_info->probed = 1;

	return ERROR_OK;
}

static int at32x_auto_probe(struct flash_bank *bank)
{
	return at32x_probe(bank);
}

static int get_at32fx_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	at32x_probe(bank);
	return ERROR_OK;
}

static int at32x_sub_bank_mass_erase(struct at32_sub_bank *sub_bank)
{
	if (sub_bank->size == 0)
		return ERROR_OK;

	ERR_CHECK(at32x_flash_unlock(sub_bank));

	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL,
				     EFCCTRL_BANKERS));
	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL,
				     EFCCTRL_BANKERS | EFCCTRL_ERSTR));

	ERR_CHECK(at32x_wait_status_busy(sub_bank, FLASH_MASS_ERASE_TIMEOUT));

	ERR_CHECK(sub_bank_write_reg(sub_bank, EFC_CTRL, EFCCTRL_OPLK));

	return ERROR_OK;
}

static int at32x_mass_erase(struct flash_bank *bank)
{
	struct at32_flash_info *at32x_info = bank->driver_priv;
	struct target *target = bank->target;
	unsigned int i;

	LOG_INFO("flash bank 0x%" PRIx32 " mass erase", (uint32_t)bank->base);
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < ARRAY_SIZE(at32x_info->sub_bank); i++) {
		struct at32_sub_bank *sub_bank = &at32x_info->sub_bank[i];
		at32x_sub_bank_mass_erase(sub_bank);
	}

	return ERROR_OK;
}


COMMAND_HANDLER(at32x_handle_mass_erase_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ERR_CHECK(CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank));

	retval = at32x_mass_erase(bank);
	if (retval == ERROR_OK)
		LOG_INFO("at32x mass erase complete");
	else
		LOG_INFO( "at32x mass erase failed");

	return retval;
}

COMMAND_HANDLER(at32x_handle_disable_access_protection_command)
{
	struct target *target = NULL;
	struct at32_flash_info *at32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	
	ERR_CHECK(CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank));

	at32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (at32x_erase_usd_data(bank) != ERROR_OK) {
		LOG_INFO("at32x failed to erase usd");
		return ERROR_OK;
	}
	at32x_info->usd_data.fap = 0xA5;
	if (at32x_write_usd_data(bank) != ERROR_OK) {
		LOG_INFO("at32x failed to write usd");
		return ERROR_OK;
	}

	LOG_INFO("AT32x disable access protection complete");

	return ERROR_OK;
}

static const struct command_registration at32f4xx_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = at32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "disable_access_protection",
		.handler = at32x_handle_disable_access_protection_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Disable read-access protection",
	},
	COMMAND_REGISTRATION_DONE
};


static const struct command_registration at32f4xx_command_handlers[] = {
	{
		.name = "at32f4xx",
		.mode = COMMAND_ANY,
		.help = "at32f4xx flash command group",
		.usage = "",
		.chain = at32f4xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


const struct flash_driver at32f4xx_flash = {
	.name = "at32f4xx",
	.commands = at32f4xx_command_handlers,
	.flash_bank_command = at32x_flash_bank_command,
	.erase = at32x_erase,
	.protect = at32x_protect,
	.write = at32x_write,
	.read = default_flash_read,
	.probe = at32x_probe,
	.auto_probe = at32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = at32x_protect_check,
	.info = get_at32fx_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 8
 * tab-width: 8
 * indent-tabs-mode: t
 * End:
 */
