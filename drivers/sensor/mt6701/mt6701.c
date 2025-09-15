/*
 * Copyright (c) 2025 Marcin Bober
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/time_units.h>
#include <zephyr/logging/log.h>

#include "mt6701.h"

LOG_MODULE_REGISTER(MT6701, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MT6701 driver enabled without any devices"
#endif

#define MT6701_FULL_ANGLE_DEG 	(360)
#define MT6701_FULL_ANGLE_RAD 	(2)
#define MT6701_RES        			(16384LL)
#define MT6701_SCALE      			(1000000LL)
#define MT6701_LAG      				(3.0f)
#define MT6701_START_UP_TIME_MS (32)


struct mt6701_config {
	union mt6701_bus bus;
	const struct mt6701_bus_io *bus_io;
};

static inline int mt6701_bus_check(const struct device *dev)
{
	const struct mt6701_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int mt6701_data_read(const struct device *dev,
				  struct mt6701_reading *buffer)
{
	const struct mt6701_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, buffer);
}

#if defined(CONFIG_MT6701_ENABLE_CRC)
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


static inline void mt6701_raw_to_angle(const int64_t raw, struct sensor_value* const val)
{
	int64_t tmp = raw * MT6701_FULL_ANGLE_DEG; 
	val->val1 = tmp / MT6701_RES;
	val->val2 = (tmp * MT6701_SCALE / MT6701_RES) % MT6701_SCALE;
}

static inline void mt6701_raw_to_radian(const int64_t raw, struct sensor_value* const val)
{
	int64_t tmp = raw * MT6701_FULL_ANGLE_RAD; 
	val->val1 = tmp / MT6701_RES;
	val->val2 = (tmp * MT6701_SCALE / MT6701_RES) % MT6701_SCALE;
}

static inline int64_t mt6701_calc_diff(const int64_t prev_position, const int64_t new_position)
{
	static const int64_t half_buffer = MT6701_RES/2;
	int64_t diff = new_position - prev_position;

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
	static const float MULT = (float)SEC_PER_MIN * Z_HZ_ticks / MT6701_RES; // SEC_PER_MIN * TICK_PER_SEC / TICK_PER_REV

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



int mt6701_sample_fetch(const struct device *dev,
				                      enum sensor_channel chan)
{
	int ret;
	struct mt6701_data *data = dev->data;
	struct mt6701_reading buffer;

  ret = mt6701_data_read(dev, &buffer);
	int64_t timestamp = k_uptime_ticks();
	uint16_t angle = (buffer.data >> 10);
	uint16_t status = (buffer.data >> 6) & 0xF;
	
	if (0 == ret)
	{
		#if defined(CONFIG_MT6701_ENABLE_CRC)
		uint8_t crc = (buffer.data & 0x3F);
		uint8_t calc_crc = mt6701_crc_calc(angle, status);
    // if (crc == calc_crc)
	#endif
    {
			uint32_t new_sample_idx = data->_sample_counter & 1;
			uint32_t old_sample_idx = new_sample_idx ^ 1;
			struct mt6701_sample* new_sample = &(data->_sample[new_sample_idx]);
			struct mt6701_sample* prev_sample = &(data->_sample[old_sample_idx]);
			new_sample->position = angle;
			new_sample->timestamp = timestamp;
			data->_status.reg = status;

			if (data->_sample_counter > 0)
			{
				data->_position_diff = mt6701_calc_diff(prev_sample->position, new_sample->position);
				data->_time_diff = new_sample->timestamp - prev_sample->timestamp;
				data->absolute_position += data->_position_diff;
			}

			data->_sample_counter++;
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
    mt6701_raw_to_angle(data->absolute_position, val);
	}
	else if (chan == SENSOR_CHAN_RPM)
	{
		float velocity = mt6701_calc_velocity(data->_position_diff, data->_time_diff);

		#if defined(CONFIG_MT6701_ENABLE_LAG)
		data->velocity = mt6701_lag(velocity, data->velocity);
		#else
		data->velocity = velocity;
		#endif
		
		sensor_value_from_float(val, data->velocity);
	}
	else {
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Sensor driver API
 */

static DEVICE_API(sensor, mt6701_api_funcs) = {
	.sample_fetch = mt6701_sample_fetch,
	.channel_get = mt6701_channel_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = mt6701_submit,
	.get_decoder = mt6701_get_decoder,
#endif /* CONFIG_SENSOR_ASYNC_API */
};

int mt6701_init(const struct device *dev)
{
	int err;

	err = mt6701_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

	k_msleep(MT6701_START_UP_TIME_MS);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

/*
 * Device instantiation macros
 */

/* Initializes a struct mt6701_config for an instance on a SPI bus. */
#define MT6701_CONFIG_SPI(inst)				\
	{						\
		.bus.spi = SPI_DT_SPEC_INST_GET(	\
			inst, MT6701_SPI_OPERATION, 0),	\
		.bus_io = &mt6701_bus_io_spi,		\
	}

/* Initializes a struct mt6701_config for an instance on an I2C bus. */
#define MT6701_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &mt6701_bus_io_i2c,	       \
	}

#define MT6701_DEFINE(inst)							\
	static struct mt6701_data mt6701_data_##inst;			\
	static const struct mt6701_config mt6701_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (MT6701_CONFIG_SPI(inst)),			\
			    (MT6701_CONFIG_I2C(inst)));			\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			 mt6701_init,				\
			 NULL,			\
			 &mt6701_data_##inst,				\
			 &mt6701_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &mt6701_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(MT6701_DEFINE)