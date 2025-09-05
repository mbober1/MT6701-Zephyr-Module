/*
 * Copyright (c) 2025 Marcin Bober
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mt6701 // TODO: add novosense

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MT6701, CONFIG_SENSOR_LOG_LEVEL);

#define MT6701_FULL_ANGLE_DEG 	(360)
#define MT6701_FULL_ANGLE_RAD 	(2)
#define MT6701_RES        			(16384LL)
#define MT6701_SCALE      			(1000000LL)
#define MT6701_LAG      				(3.0f)

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
	bool last_fetch_success;
	float velocity;

	uint_fast16_t position;
	int64_t last_read_timestamp;
  union mt6701_status status;
};

uint8_t mt6701_crc_calc(uint_fast16_t angle, uint8_t mag) {
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

static inline void mt6701_raw_to_angle(const uint_fast16_t raw, struct sensor_value* const val)
{
	int_fast64_t tmp = (int_fast64_t)raw * MT6701_FULL_ANGLE_DEG; 
	val->val1 = tmp / MT6701_RES;
	val->val2 = (tmp * MT6701_SCALE / MT6701_RES) % MT6701_SCALE;
}

static inline void mt6701_raw_to_radian(const uint_fast16_t raw, struct sensor_value* const val)
{
	int_fast64_t tmp = (int_fast64_t)raw * MT6701_FULL_ANGLE_RAD; 
	val->val1 = tmp / MT6701_RES;
	val->val2 = (tmp * MT6701_SCALE / MT6701_RES) % MT6701_SCALE;
}

static inline int_fast16_t mt6701_calc_diff(const int_fast16_t prev_position, const int_fast16_t new_position)
{
	static const int_fast16_t half_buffer = MT6701_RES/2;
	int_fast16_t diff = new_position - prev_position;

	if (diff > half_buffer)
	{
		diff = diff - MT6701_RES;
	}
	else if (diff < -half_buffer)
	{
		diff = diff + MT6701_RES;
	}
	
	return diff;
}

static inline float mt6701_calc_velocity(const float diff, const float ticks)
{
	static const float MULT = 60.0f * Z_HZ_ticks / MT6701_RES; // SEC_IN_MIN * TICK_PER_SER * TICK_PER_REV

	if (ticks > 0)
	{
		return diff * MULT / ticks; // [RPM]
	}
	else
	{
		return 0;
	}
}

static inline float mt6701_lag(const float new_val, const float old_val)
{
	static const float new_mul = MT6701_LAG / 100;
	static const float old_mult = 1.f - new_mul;

	return old_mult * old_val + new_mul * new_val;
}



static inline int mt6701_sample_fetch(const struct device *dev,
				                      enum sensor_channel chan)
{
	int ret;
	struct mt6701_data *data = dev->data;
	struct mt6701_payload buffer;

  ret = mt6701_read(dev, &buffer);
	int64_t timestamp = k_uptime_ticks();

	if (0 == ret) { // TODO: fix CRC
		// uint8_t calc_crc = mt6701_crc_calc(buffer.angle, buffer.status);

    // if (buffer.crc == calc_crc)
    {
			uint_fast16_t new_position = buffer.angle;

			if (true == data->last_fetch_success)
			{
				uint_fast16_t prev_position = data->position;
				int_fast16_t position_diff = mt6701_calc_diff(prev_position, new_position);
				int64_t time_diff = timestamp - data->last_read_timestamp;
				float new_velocity = mt6701_calc_velocity(position_diff, time_diff);
				data->velocity = mt6701_lag(new_velocity, data->velocity);
			}
			
			data->position = new_position;
			data->status.raw = buffer.status;
			data->last_read_timestamp = timestamp;
			data->last_fetch_success = true;
    }
	}

	return ret;
}

static int mt6701_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct mt6701_data *data = dev->data;

	if (chan == SENSOR_CHAN_ROTATION)
	{
    mt6701_raw_to_angle(data->position, val);
	}	
	else if (chan == SENSOR_CHAN_RPM)
	{
		sensor_value_from_float(val, data->velocity);
	}
	else {
		return -ENOTSUP;
	}

	return 0;
}

static DEVICE_API(sensor, mt6701_api) = {
	.sample_fetch = &mt6701_sample_fetch,
	.channel_get = &mt6701_channel_get,
};

int mt6701_init(const struct device *dev)
{
	const struct mt6701_config *config = dev->config;
	struct mt6701_data *data = dev->data;

	data->last_fetch_success = false;
	data->velocity = 0.0f;
	data->position = 0;
	data->last_read_timestamp = 0;
	data->status.raw = 0;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	LOG_INF("Device %s initialized", dev->name);

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