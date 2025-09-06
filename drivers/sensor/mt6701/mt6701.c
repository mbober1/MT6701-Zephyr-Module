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
	#if defined(CONFIG_MT6701_ENABLE_CRC)
  uint32_t status: 8;
  uint32_t crc: 6;
  uint32_t test: 11; // TODO
	#endif
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

struct mt6701_sample {
	uint_fast16_t position;
	int64_t timestamp;
};

struct mt6701_data {
	float velocity;

	uint_fast32_t sample_counter;
	struct mt6701_sample sample[2];
  union mt6701_status status;
};

#if defined(CONFIG_MT6701_CRC)
// CRC-6 lookup table for polynomial X^6 + X^1 + 1
const uint8_t crcTable[64] = {
	0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
	0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
	0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
	0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
	0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
	0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
	0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
	0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
};

uint8_t mt6701_crc_calc(uint_fast16_t angle, uint8_t mag) {
	uint32_t data = ((uint32_t)angle << 4) | (mag & 0x0F); // 18-bit stream
	// Combine the 14-bit angle data and 4-bit status into 18 bits
	uint32_t combinedData = data >> 6;  // Shift out the 6-bit CRC
	uint8_t crc = 0;
	// Process the data 6 bits at a time (from MSB to LSB)
	crc = crcTable[(crc ^ (combinedData >> 12)) & 0x3F];  // First 6 bits
	crc = crcTable[(crc ^ (combinedData >> 6)) & 0x3F];   // Second 6 bits
	crc = crcTable[(crc ^ combinedData) & 0x3F];          // Last 6 bits
	return crc;
}
#endif

static inline int mt6701_read(const struct device *dev, 
                      struct mt6701_payload *buffer)
{
	const struct mt6701_config *config = dev->config;

	const struct spi_buf rx_buf = {
		.buf = buffer,
		.len = sizeof(struct mt6701_payload)
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

	if (0 == ret) 
	{

	#if defined(CONFIG_MT6701_ENABLE_CRC)
		uint8_t calc_crc = mt6701_crc_calc(buffer.angle, buffer.status);
			// data->status.raw = buffer.status;
    if (buffer.crc == calc_crc)
	#endif
    {
			uint_fast16_t new_sample = data->sample_counter & 1;
			data->sample[new_sample].position = buffer.angle;
			data->sample[new_sample].timestamp = timestamp;
			data->sample_counter++;
    }
	}

	return ret;
}

static int mt6701_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct mt6701_data *data = dev->data;
	uint_fast16_t prev_sample = data->sample_counter & 1;
	uint_fast16_t latest_sample = prev_sample ^ 1;

	if (chan == SENSOR_CHAN_ROTATION)
	{
    mt6701_raw_to_angle(data->sample[latest_sample].position, val);
	}
	else if (chan == SENSOR_CHAN_RPM)
	{
		if (data->sample_counter > 1)
		{
			uint_fast16_t prev_position = data->sample[prev_sample].position;
			int64_t prev_timestamp = data->sample[prev_sample].timestamp;

			uint_fast16_t latest_position = data->sample[latest_sample].position;
			int64_t latest_timestamp = data->sample[latest_sample].timestamp;

			int_fast16_t position_diff = mt6701_calc_diff(prev_position, latest_position);
			int64_t time_diff = latest_timestamp - prev_timestamp;

			float velocity = mt6701_calc_velocity(position_diff, time_diff);

			#if defined(CONFIG_MT6701_ENABLE_LAG)
			data->velocity = mt6701_lag(velocity, data->velocity);
			#else
			data->velocity = velocity;
			#endif
			
			sensor_value_from_float(val, data->velocity);
		}
		else
		{
			val->val1 = 0;
			val->val2 = 0;
		}
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

	data->velocity = 0.0f;
	data->sample_counter = 0;
	data->status.raw = 0;
	memset(data->sample, 0, sizeof(data->sample));

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