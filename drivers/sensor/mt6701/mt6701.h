/*
 * Copyright (c) 2025 Marcin Bober
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MT6701_MT6701_H_
#define ZEPHYR_DRIVERS_SENSOR_MT6701_MT6701_H_

#include <zephyr/drivers/sensor.h>

#define DT_DRV_COMPAT novosense_mt6701

#define MT6701_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define MT6701_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)


#if MT6701_BUS_SPI
#include <zephyr/drivers/spi.h>
#endif

#if MT6701_BUS_I2C
#include <zephyr/drivers/i2c.h>
#endif

#ifdef CONFIG_MT6701_STREAM
#include <zephyr/rtio/rtio.h>
#endif /* CONFIG_MT6701_STREAM */


union mt6701_bus {
#if MT6701_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if MT6701_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

struct mt6701_reading {
	uint32_t data;
};

struct mt6701_status_fields {
  uint8_t field_status: 2;
  uint8_t push_button_detect: 1;
  uint8_t track_loss: 1;
} __attribute__((__packed__));

struct mt6701_sample {
	uint_fast16_t position;
	int64_t timestamp;
};

typedef int (*mt6701_bus_check_fn)(const union mt6701_bus *bus);
typedef int (*mt6701_data_read_fn)(const union mt6701_bus *bus, struct mt6701_reading *buffer);

struct mt6701_bus_io {
	mt6701_bus_check_fn check;
	mt6701_data_read_fn read;
};

// TODO: test this on 8bit transactions!!!
#if MT6701_BUS_SPI
#define MT6701_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)
extern const struct mt6701_bus_io mt6701_bus_io_spi;
#endif

#if MT6701_BUS_I2C
extern const struct mt6701_bus_io mt6701_bus_io_i2c;
#endif


union mt6701_status {
  struct mt6701_status_fields status;
  uint8_t reg;
};

struct mt6701_data {
	float velocity;
	int64_t absolute_position;

	uint_fast32_t _sample_counter;
	int64_t _time_diff;
	int64_t _position_diff;
	struct mt6701_sample _sample[2];
  union mt6701_status _status;
};


/*
 * RTIO
 */

struct mt6701_decoder_header {
	uint64_t timestamp;
} __attribute__((__packed__));

// struct mt6701_encoded_data {
// 	struct mt6701_decoder_header header;
// 	struct {
// 		/** Set if `temp` has data */
// 		uint8_t has_temp: 1;
// 		/** Set if `press` has data */
// 		uint8_t has_press: 1;
// 		/** Set if `humidity` has data */
// 		uint8_t has_humidity: 1;
// 	} __attribute__((__packed__));
// 	struct mt6701_reading reading;
// };

int mt6701_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

void mt6701_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe);

int mt6701_sample_fetch(const struct device *dev, enum sensor_channel chan);

#endif /* ZEPHYR_DRIVERS_SENSOR_MT6701_MT6701_H_ */