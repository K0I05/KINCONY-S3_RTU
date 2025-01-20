/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file modbus_connect.h
 *
 * MODBUS master connection to RS2E RTU libary
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MODBUS_CONNECT_H__
#define __MODBUS_CONNECT_H__

#include <stdint.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_types.h>
#include <mbcontroller.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets temperature over MODBUS from RS2E RTU.
 * 
 * @param value Temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_rs2e_temperature(float *value);

/**
 * @brief Gets relative humidity over MODBUS from RS2E RTU.
 * 
 * @param value Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_rs2e_humidity(float *value);

/**
 * @brief Gets precipitation type over MODBUS from RS2E RTU.
 * 
 * @param value Precipitation type as a code, see user manual for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_rs2e_precipitation_type(uint16_t *value);

/**
 * @brief Gets precipitation rate over MODBUS from RS2E RTU.
 * 
 * @param value Precipitation rate in millimeters per hour.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_rs2e_precipitation_rate(float *value);

/**
 * @brief Gets wind direction (instantanious) over MODBUS from WDC2E RTU.
 * 
 * @param value Wind direction in degrees.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_wdc2e_wind_direction(uint16_t *value);

/**
 * @brief Gets wind speed (instantanious) over MODBUS from WDC2E RTU.
 * 
 * @param value Wind speed in meters per second.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_wdc2e_wind_speed(float *value);

/**
 * @brief Gets temperature over MODBUS from SHT30 RTU.
 * 
 * @param value Temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_sht30_temperature(float *value);

/**
 * @brief Gets relative humidity over MODBUS from SHT30 RTU.
 * 
 * @param value Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_get_sht30_humidity(float *value);

/**
 * @brief Initializes MODBUS master controller.
 * 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t master_init(void);


#ifdef __cplusplus
}
#endif

#endif // __MODBUS_CONNECT_H__