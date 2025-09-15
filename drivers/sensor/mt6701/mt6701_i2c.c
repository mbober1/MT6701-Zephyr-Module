/*
 * Copyright (c) 2025 Marcin Bober
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for MT6701s accessed via I2C.
 */

#include "mt6701.h"

#if MT6701_BUS_I2C
static int mt6701_bus_check_i2c(const union mt6701_bus *bus)
{
	return -ENOTSUP;
}

static int mt6701_data_read_i2c(const union mt6701_bus *bus,
			       uint8_t start, uint8_t *buf, int size)
{
	return -ENOTSUP;
}

static int mt6701_reg_write_i2c(const union mt6701_bus *bus,
				uint8_t reg, uint8_t val)
{
	return -ENOTSUP;
}

const struct mt6701_bus_io mt6701_bus_io_i2c = {
	.check = mt6701_bus_check_i2c,
	.read = mt6701_data_read_i2c,
	.write = mt6701_reg_write_i2c,
};
#endif /* MT6701_BUS_I2C */
