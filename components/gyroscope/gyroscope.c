#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>
#include <math.h>

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

static const char *TAG = "mpu6050_test";
mpu6050_dev_t dev = { 0 };

void mpu6050_get_pitch_roll(float *pitch, float *roll)
{
    float temp;
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };
    
    static float pitch_est = 0.0f, roll_est = 0.0f;
    const float alpha = 0.7f;  // peso do giroscópio
    const float dt = 0.05f;     // tempo entre leituras (em s) — deve coincidir com o vTaskDelay()

    float ax, ay, az;
    float gx, gy, gz;

    ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
    ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

    az = accel.x;
    ay = -accel.y;
    ax = accel.z;

    gz = rotation.x;
    gy = -rotation.y;
    gx = rotation.z;

    // Normaliza vetor de aceleração
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0) norm = 1;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Ângulos vindos do acelerômetro
    float pitch_acc = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    float roll_acc  = atan2f(-ay, az) * 180.0f / M_PI;

    // Integra giroscópio (em °/s)
    pitch_est += gx * dt;
    roll_est  += gy * dt;

    // Filtro complementar (mistura gyro + accel)
    pitch_est = alpha * pitch_est + (1.0f - alpha) * pitch_acc;
    roll_est  = alpha * roll_est  + (1.0f - alpha) * roll_acc;

    *pitch = pitch_est;
    *roll  = roll_est;
}

void mpu6050_test(void *pvParameters)
{
    //mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    //ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    //ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);
    vTaskDelete(NULL);
    /*
    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        
        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        ESP_LOGI(TAG, "Temperature:  %.1f", temp);
        

        vTaskDelay(pdMS_TO_TICKS(100));
    }*/
}

void gyro_init()
{
    // task
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

