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
	const char *mmc_id;             // REQUIRED for SDIO
	int spi_bits_per_word;          // OPTIONAL only for SPI
	unsigned int pll_init_val;      // REQUIRED
	int disable_irq;                // set to 1 as needed.
	const struct resource *irq;     // REQUIRED when GPIO_IRQ is in use
	const struct resource *reset;   // Can be NULL w/ HW circuit
	int (*power_ctrl)(const struct cw1200_platform_data *pdata,
			  bool enable); // Can be NULL w/ H/W reset circuit
	int (*clk_ctrl)(const struct cw1200_platform_data *pdata,
			bool enable); // Can be NULL
};

/* Declaration only. Should be implemented in arch/xxx/mach-yyy */
const struct cw1200_platform_data *cw1200_get_platform_data(void);

#endif /* CW1200_PLAT_H_INCLUDED */
