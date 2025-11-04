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
    /**
     *   / _ \ / __\/ _ \  /_\  / _ \/ |
     *  / /_\// /  | (_) |//_\\| | | | |
     * / /_\\/ /___ \__, /  _  \ |_| | |
     * \____/\____/   /_/\_/ \_/\___/|_|
     */
    printf("   ___   ___  ___    _    ___  _\r\n");
    printf("  / _ \\ / __\\/ _ \\  /_\\  / _ \\/ |\r\n");
    printf(" / /_\\// /  | (_) |//_\\\\| | | | |\r\n");
    printf("/ /_\\\\/ /___ \\__, /  _  \\ |_| | |\r\n");
    printf("\\____/\\____/   /_/\\_/ \\_/\\___/|_|\r\n");
    tester();
}

