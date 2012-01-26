/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef CW1200_PLAT_H_INCLUDED
#define CW1200_PLAT_H_INCLUDED

#include <linux/ioport.h>

struct cw1200_platform_data {
	const char *mmc_id;             // REQUIRED, SDIO-only
	int spi_bits_per_word;          // REQUIRED, SPI-only
	unsigned int pll_init_val;      // REQUIRED
	int disable_irq;                // set to 1 as needed
	const struct resource *irq;     // REQUIRED for SPI or SDIO w/GPIO_IRQ
	const struct resource *reset;   // Can be NULL w/ HW reset circuit
	int (*power_ctrl)(const struct cw1200_platform_data *pdata,
			  bool enable); // Can be NULL w/ H/W power circuit
	int (*clk_ctrl)(const struct cw1200_platform_data *pdata,
			bool enable); // Can be NULL
	const u8 *macaddr;  // if NULL, use cw1200_mac_template module parameter 
};

/* Declaration only. Should be implemented in arch/xxx/mach-yyy */
const struct cw1200_platform_data *cw1200_get_platform_data(void);

#endif /* CW1200_PLAT_H_INCLUDED */
