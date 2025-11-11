#include <stdio.h>
#include "drawer.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <inttypes.h>
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
//#include "esp_lcd_panel_vendor.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "esp_lcd_gc9a01.h"
#include "math.h"
#include <string.h>
#include "gyroscope.h"

float pitch = 0.0f;
float roll = 0.0f;

#define TEST_LCD_HOST               SPI2_HOST
#define TEST_LCD_H_RES              (240)
#define TEST_LCD_V_RES              (320)
#define TEST_LCD_BIT_PER_PIXEL      (16)
//#define TEST_LCD_BYTE_PER_PIXEL      (TEST_LCD_BIT_PER_PIXEL/8)
#define TEST_PIN_NUM_LCD_CS         (GPIO_NUM_16)
#define TEST_PIN_NUM_LCD_PCLK       (GPIO_NUM_19)
#define TEST_PIN_NUM_LCD_DATA0      (GPIO_NUM_23)
#define TEST_PIN_NUM_LCD_RST        (GPIO_NUM_4)
#define TEST_PIN_NUM_LCD_DC         (GPIO_NUM_17)

// Cores em RGB565
#define COLOR_SKY   0x4AD4   // Azul/Céu
#define COLOR_EARTH 0xBE03    // Marrom/Terra
#define COLOR_WHITE 0x0000    // Branco/Ticks


#define TEST_DELAY_TIME_MS          (3000)

static char *TAG = "drawing";

static SemaphoreHandle_t refresh_finish = NULL;


IRAM_ATTR static bool test_notify_refresh_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}

static void draw_line(uint16_t *color_buf, int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (true) {
        if (x0 >= 0 && x0 < TEST_LCD_H_RES && y0 >= 0 && y0 < TEST_LCD_V_RES)
            color_buf[y0 * TEST_LCD_H_RES + x0] = color;
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_horizon(uint16_t *color, float pitch_deg, float roll_deg)
{
    int width = TEST_LCD_H_RES;
    int height = TEST_LCD_V_RES;

    // converte ângulos para radianos
    float pitch = pitch_deg * M_PI / 180.0f;
    float roll  = roll_deg  * M_PI / 180.0f;

    // centro da tela
    float cx = width / 2.0f;
    float cy = height / 2.0f;

    // sensibilidade do pitch: quantos pixels por grau
    float pixels_per_deg = height / 60.0f; // horizonte move ~60° de pitch no total

    // deslocamento vertical devido ao pitch
    float y_offset = pitch_deg * pixels_per_deg;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Coordenadas relativas ao centro
            float dx = x - cx;
            float dy = y - cy + y_offset;

            // Rota o ponto de acordo com roll
            float xr = dx * cosf(roll) - dy * sinf(roll);
            float yr = dx * sinf(roll) + dy * cosf(roll);

            // Se yr < 0 → céu, senão → terra
            uint16_t color_pixel = (yr < 0) ? COLOR_SKY : COLOR_EARTH;
            color[y * width + x] = color_pixel;
        }
    }
    // --- Linha central ---
    draw_line(color, 0, cy, TEST_LCD_H_RES - 1, cy, COLOR_WHITE);

    // --- Tiques horizontais ---
    for (int i = -30; i <= 30; i += 10) {
        if (i == 0) continue;  // linha central já desenhada
        int offset = (int)(i * (TEST_LCD_V_RES / 90.0f));
        int y_line = cy + offset;
        if (y_line < 0 || y_line >= TEST_LCD_V_RES) continue;

        int len = 20 - abs(i) / 2;
        draw_line(color, cx - len, y_line, cx + len, y_line, COLOR_WHITE);
    }

    // --- Linha vertical central ---
    draw_line(color, cx, cy - 20, cx, cy + 20, COLOR_WHITE);

}

// Função principal: desenha horizonte artificial
void draw_artificial_horizon(esp_lcd_panel_handle_t panel_handle, float pitch_deg, float roll_deg)
{
    refresh_finish = xSemaphoreCreateBinary();
    if (refresh_finish == NULL) return;

    uint8_t byte_per_pixel = TEST_LCD_BIT_PER_PIXEL / 8;
    uint16_t *color = (uint16_t *)heap_caps_malloc(TEST_LCD_V_RES * TEST_LCD_H_RES * byte_per_pixel, MALLOC_CAP_DMA);
    if (color == NULL) return;
    
    // Gera imagem do horizonte
    draw_horizon(color, pitch_deg, roll_deg);

    // Envia buffer inteiro ao painel
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, TEST_LCD_H_RES, TEST_LCD_V_RES, color);

    // Espera atualização terminar (caso callback use refresh_finish)
    xSemaphoreTake(refresh_finish, pdMS_TO_TICKS(200));

    free(color);
    vSemaphoreDelete(refresh_finish);
}

void run_display(void)
{
    
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(TEST_PIN_NUM_LCD_PCLK, TEST_PIN_NUM_LCD_DATA0,
                                    TEST_LCD_H_RES * 80 * TEST_LCD_BIT_PER_PIXEL / 8);
    TEST_ESP_OK(spi_bus_initialize(TEST_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(TEST_PIN_NUM_LCD_CS, TEST_PIN_NUM_LCD_DC,
            test_notify_refresh_ready, NULL);
    // Attach the LCD to the SPI bus
    TEST_ESP_OK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TEST_LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_PIN_NUM_LCD_RST,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
#elif ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
#else
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
#endif
        .bits_per_pixel = TEST_LCD_BIT_PER_PIXEL,
    };
    
    TEST_ESP_OK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    TEST_ESP_OK(esp_lcd_panel_reset(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_init(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_invert_color(panel_handle, true));
    TEST_ESP_OK(esp_lcd_panel_mirror(panel_handle, true, false));
    
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    TEST_ESP_OK(esp_lcd_panel_disp_off(panel_handle, false));
#else
    TEST_ESP_OK(esp_lcd_panel_disp_on_off(panel_handle, true));
#endif

    //test_draw_bitmap(panel_handle);
    while (true) {
        mpu6050_get_pitch_roll (&pitch, &roll);
        draw_artificial_horizon (panel_handle, pitch, roll);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_TIME_MS));

    TEST_ESP_OK(esp_lcd_panel_del(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_io_del(io_handle));
    TEST_ESP_OK(spi_bus_free(TEST_LCD_HOST));
}

// Some resources are lazy allocated in the LCD driver, the threadhold is left for that case
#define TEST_MEMORY_LEAK_THRESHOLD (-300)

static size_t before_free_8bit;
static size_t before_free_32bit;

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");
}

void drawer_init(void)
{
    run_display();    
}
