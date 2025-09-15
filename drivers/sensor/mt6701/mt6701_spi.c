/*
 * Copyright (c) 2025 Marcin Bober
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for MT6701s accessed via SPI.
 */

#include <zephyr/logging/log.h>
#include "mt6701.h"
#include <zephyr/sys/byteorder.h>

#if MT6701_BUS_SPI

LOG_MODULE_DECLARE(MT6701, CONFIG_SENSOR_LOG_LEVEL);

static int mt6701_bus_check_spi(const union mt6701_bus *bus)
{
	return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

static inline int mt6701_data_read_spi(const union mt6701_bus *bus, 
                      struct mt6701_reading *out)
{
	uint8_t buffer[3];
	const struct spi_buf rx_buf = {
		.buf = buffer,
		.len = ARRAY_SIZE(buffer),
	};

	const struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1U
	};

	int ret = spi_read_dt(&bus->spi, &rx_bufs);

	uint32_t value = 	((uint32_t)buffer[0] << 16) |
										((uint32_t)buffer[1] << 8)  |
										((uint32_t)buffer[2]);
	
	out->data = 	value;

	return ret;
}

const struct mt6701_bus_io mt6701_bus_io_spi = {
	.check = mt6701_bus_check_spi,
	.read = mt6701_data_read_spi,
};
#endif /* MT6701_BUS_SPI */
