/*
 * Copyright (c) 2025 Marcin Bober
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mt6701 // TODO: add novosense

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MT6701, CONFIG_SENSOR_LOG_LEVEL);

struct mt6701_config {
	struct spi_dt_spec spi;
};

struct mt6701_payload {
  uint32_t : 1;
  uint32_t angle: 14;
  uint32_t status: 4;
  uint32_t crc: 6;
} __attribute__((__packed__));

struct mt6701_status_fields {
  uint8_t field_status: 2;
  uint8_t push_button_detect: 1;
  uint8_t track_loss: 1;
} __attribute__((__packed__));

union mt6701_status {
  struct mt6701_status_fields status;
  uint8_t raw;
};

struct mt6701_data {
	uint16_t raw_angle;
  union mt6701_status status;
};

uint8_t mt6701_crc_calc(uint16_t angle, uint8_t mag) {
    uint32_t data = ((uint32_t)angle << 4) | (mag & 0x0F); // 18-bit stream
    uint8_t crc = 0;
    const uint8_t poly = 0x43; // x^6 + x + 1

    for (int i = 17; i >= 0; i--) { // MSB-first
        uint8_t bit = (data >> i) & 0x01;
        uint8_t feedback = bit ^ ((crc >> 5) & 0x01);
        crc = (crc << 1) & 0x3F;
        if (feedback) {
            crc ^= poly;
        }
    }

    return crc;
}

static inline int mt6701_read(const struct device *dev, 
                      struct mt6701_payload *buffer)
{
	const struct mt6701_config *config = dev->config;

	const struct spi_buf rx_buf = {
		.buf = buffer,
		.len = 4U
	};

	const struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1U
	};

	return spi_read_dt(&config->spi, &rx_bufs);
}

static inline int mt6701_sample_fetch(const struct device *dev,
				                      enum sensor_channel chan)
{
	int ret;
	struct mt6701_data *data = dev->data;
	struct mt6701_payload buffer;

  ret = mt6701_read(dev, &buffer);

	if (0 == ret) {
		// uint8_t calc_crc = mt6701_crc_calc(buffer.angle, buffer.status);

    // if (buffer.crc == calc_crc)
    {
      data->raw_angle = buffer.angle;
      data->status.raw = buffer.status;
    }
	}

	return ret;
}

static int mt6701_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	// struct mt6701_data *data = dev->data;

	// if (chan == SENSOR_CHAN_ROTATION) {
  //   LOG_INF("");
	// 	// val->val1 = ((int32_t)dev_data->position * AS5600_FULL_ANGLE) /
	// 	// 					AS5600_PULSES_PER_REV;

	// 	// val->val2 = (((int32_t)dev_data->position * AS5600_FULL_ANGLE) %
	// 	// 	     AS5600_PULSES_PER_REV) * (AS5600_MILLION_UNIT / AS5600_PULSES_PER_REV);
	// } else {
	// 	return -ENOTSUP;
	// }

	return 0;
}

static DEVICE_API(sensor, mt6701_api) = {
	.sample_fetch = &mt6701_sample_fetch,
	.channel_get = &mt6701_channel_get,
};

int mt6701_init(const struct device *dev)
{
	const struct mt6701_config *config = dev->config;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	return 0;
}

#define MT6701_INIT(n)							\
	static struct mt6701_data mt6701_data_##n;			\
	static const struct mt6701_config mt6701_config_##n = {	\
		.spi = SPI_DT_SPEC_INST_GET(n,				\
					    SPI_OP_MODE_MASTER |	\
					    SPI_TRANSFER_MSB |		\
					    SPI_MODE_CPOL |		\
					    SPI_MODE_CPHA |		\
					    SPI_WORD_SET(16U),		\
					    0U),			\
	};								\
	SENSOR_DEVICE_DT_INST_DEFINE(n, &mt6701_init, NULL,		\
			      &mt6701_data_##n, &mt6701_config_##n,	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
			      &mt6701_api);

DT_INST_FOREACH_STATUS_OKAY(MT6701_INIT)