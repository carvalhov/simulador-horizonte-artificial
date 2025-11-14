#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include "gyroscope.h"
#include "drawer.h"
#include "esp_lcd_gc9a01.h"

static char *TAG = "main";

void app_main(void)
{
    gyro_init();
    drawer_init();
}

