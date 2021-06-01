/* 
* This file is part of VL53L0X Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "vl53l0X_platform.h"
#include "vl53l0X_platform_log.h"
#include "vl53l0X_api.h"

//#define I2C_PORT                        I2C_NUM_1
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

uint8_t _I2CBuffer[256];

int
_I2CWrite(VL53L0X_DEV Dev, uint8_t *buf, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (Dev->I2cDevAddr|WRITE_BIT), ACK_CHECK_EN);
    i2c_master_write(cmd, buf, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(*(Dev->I2cHandle), cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return -1;
    }
    return 0;
}

int
_I2CRead(VL53L0X_DEV Dev, uint8_t *buf, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (Dev->I2cDevAddr|READ_BIT), ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(*(Dev->I2cHandle), cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return -1;
    }
    return 0;
}

static void
VL53L0X_GetI2cBus(void)
{
}

static void
VL53L0X_PutI2cBus(void)
{
}

VL53L0X_Error
VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    int status_int;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index;
    memcpy(&_I2CBuffer[1], pdata, count);
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, count + 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error
VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, pdata, count);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error
VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error
VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data >> 8;
    _I2CBuffer[2] = data & 0x00FF;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error
VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index;
    _I2CBuffer[1] = (data >> 24) & 0xFF;
    _I2CBuffer[2] = (data >> 16) & 0xFF;
    _I2CBuffer[3] = (data >> 8)  & 0xFF;
    _I2CBuffer[4] = (data >> 0 ) & 0xFF;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 5);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error
VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L0X_Error
VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

	_I2CBuffer[0] = index;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 1);
    if( status_int ){
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, data, 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error
VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 1);

    if( status_int ){
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error
VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // do nothing
    vTaskDelay( 5/portTICK_PERIOD_MS);
    //VL53L0X_OsDelay();

    return status;
}
