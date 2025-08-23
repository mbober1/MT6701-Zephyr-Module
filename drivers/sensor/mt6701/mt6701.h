// /*
//  * Copyright (c) 2025 Marcin Bober
//  *
//  * SPDX-License-Identifier: Apache-2.0
//  */

// #ifndef ZEPHYR_DRIVERS_SENSOR_MT6701_MT6701_H_
// #define ZEPHYR_DRIVERS_SENSOR_MT6701_MT6701_H_

// #include <zephyr/types.h>
// #include <zephyr/drivers/spi.h>


// #include <zephyr/device.h>
// #include <zephyr/sys/util.h>
// #include <zephyr/types.h>
// #include <zephyr/drivers/i2c.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/kernel.h>

// // #define MT6701_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

// // union mt6701_bus {
// // 	struct spi_dt_spec spi;
// // };

// // typedef int (*mt6701_bus_check_fn)(const union mt6701_bus *bus);
// // typedef int (*mt6701_reg_read_fn) (const union mt6701_bus *bus, uint8_t start, uint8_t *buf, int size);
// // typedef int (*mt6701_reg_write_fn)(const union mt6701_bus *bus, uint8_t reg, uint8_t val);

// // struct mt6701_bus_io {
// // 	mt6701_bus_check_fn check;
// // 	mt6701_reg_read_fn read;
// // 	mt6701_reg_write_fn write;
// // };

// // #define MT6701_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)
// // extern const struct mt6701_bus_io mt6701_bus_io_spi;




// #endif /* ZEPHYR_DRIVERS_SENSOR_MT6701_MT6701_H_ */