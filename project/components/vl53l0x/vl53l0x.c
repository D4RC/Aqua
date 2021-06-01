#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"
#include <esp_log.h>
#include <esp_log_internal.h>

#include "vl53l0x_api.h"

VL53L0X_Dev_t dev;
VL53L0X_DEV Dev = &dev;

static int i2c_handle = CONFIG_I2C_NUM;

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

void vl53l0x_init()
{
    Dev->I2cHandle = &i2c_handle;
    Dev->I2cDevAddr = 0x52;
    static const char *TAG = "VL53L0X";

    VL53L0X_DeviceInfo_t DevInfo;
    int status = 0;
    
    status = VL53L0X_DataInit(Dev);
    if(status)
        ESP_LOGI(TAG, "VL53L0X DataInit Failed");

    status = VL53L0X_StaticInit(Dev);
    if(status)
        ESP_LOGI(TAG, "VL53L0X StaticInit Failed");

    uint8_t isApertureSpads;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t VhvSettings;

    status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    if (status != VL53L0X_ERROR_NONE)
        ESP_LOGI(TAG, "VL53L0X PerformRefSpadManagement Failed");
    else
        ESP_LOGI(TAG, "refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);

    status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE)
        ESP_LOGI(TAG, "VL53L0X PerformRefCalibration Failed");
    else
        ESP_LOGI(TAG, "VhvSettings = %d, PhaseCal = %d\n", VhvSettings, PhaseCal);


    status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if(status)
        ESP_LOGI(TAG, "VL53L0X SetDeviceMode Failed");

    status = VL53L0X_SetGpioConfig(Dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH, VL53L0X_INTERRUPTPOLARITY_LOW);
    if(status)
        ESP_LOGI(TAG, "VL53L0X SetGpioConfig Failed");

    status = VL53L0X_SetInterruptThresholds(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, (400*65536), (400*65536));
    if(status)
        ESP_LOGI(TAG, "VL53L0X SetInterruptThresholds Failed");

    // status = VL53L0X_set_gpio(1);
    // if(status)
    //     ESP_LOGI(TAG, "VL53L0X set_gpio Failed");

    status = VL53L0X_GetDeviceInfo(Dev, &DevInfo);
    if(status)
        ESP_LOGI(TAG, "VL53L0X GetDeviceInfo Failed");

    printf("VL53L0X Device Info:\n");
    printf("Device Name : %s\n", DevInfo.Name);
    printf("Device Type : %s\n", DevInfo.Type);
    printf("Device ID : %s\n", DevInfo.ProductId);
    printf("ProductRevisionMajor : %d\n", DevInfo.ProductRevisionMajor);
    printf("ProductRevisionMinor : %d\n", DevInfo.ProductRevisionMinor);

    FixPoint1616_t thigh, tlow;
    status = VL53L0X_GetInterruptThresholds(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, &tlow, &thigh);
    if(status)
        ESP_LOGI(TAG, "VL53L0X GetThresholds Failed");
    
    printf("Thresholds: %.2f,%.2f\n", (tlow/65536.0), thigh/65536.0);

    status = VL53L0X_StartMeasurement(Dev);
    if(status)
        ESP_LOGI(TAG, "VL53L0X StartMeasurement Failed");
}
