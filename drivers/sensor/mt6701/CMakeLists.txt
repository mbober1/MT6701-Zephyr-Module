# Copyright (c) 2025 Marcin Bober
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(mt6701.c mt6701_spi.c mt6701_i2c.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API mt6701_decoder.c mt6701_async.c)
