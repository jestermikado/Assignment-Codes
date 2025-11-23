#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include <math.h>

#define I2C_MASTER_CLOCK 22
#define I2C_MASTER_DATA 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TIMEOUT_MS 1000 / portTICK_PERIOD_MS

#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_data;

static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t out[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, out, sizeof(out), I2C_TIMEOUT_MS);
}

static esp_err_t i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

esp_err_t mpu_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_DATA,
        .scl_io_num = I2C_MASTER_CLOCK,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return i2c_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
}

esp_err_t mpu_whoami(uint8_t *id_out)
{
    if (id_out == NULL) return ESP_ERR_INVALID_ARG;
    return i2c_read_regs(MPU6050_ADDR, MPU6050_WHO_AM_I, id_out, 1);
}

esp_err_t mpu_read_accel(sensor_data *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t buf[6];
    esp_err_t err = i2c_read_regs(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    return ESP_OK;
}

esp_err_t mpu_read_gyro(sensor_data *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t buf[6];
    esp_err_t err = i2c_read_regs(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

    err = mpu_init();
    if (err != ESP_OK) {
        printf("MPU init failed: %s\n", esp_err_to_name(err));
        return;
    }

    uint8_t who = 0;
    err = mpu_whoami(&who);
    if (err == ESP_OK) {
        printf("MPU WHO_AM_I = 0x%02X\n", who);
    } else {
        printf("WHO_AM_I read failed: %s\n", esp_err_to_name(err));
    }

    sensor_data accel, gyro;
    while (1) {
        if (mpu_read_accel(&accel) == ESP_OK) {
            printf("Accel raw data: X=%d Y=%d Z=%d\n", accel.x, accel.y, accel.z);
        } else {
            printf("Accel read error\n");
        }

        if (mpu_read_gyro(&gyro) == ESP_OK) {
            printf("Gyro  raw data: X=%d Y=%d Z=%d\n", gyro.x, gyro.y, gyro.z);
        } else {
            printf("Gyro read error\n");
        }

        printf("\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
