#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C Configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0  // GP0 (SDA)
#define I2C_SCL_PIN 1  // GP1 (SCL)
#define LSM303_ACCEL_ADDR 0x19   // LSM303DLHC accelerometer I2C address

// LSM303DLHC Accelerometer Registers
#define LSM303_CTRL_REG1_A    0x20
#define LSM303_CTRL_REG4_A    0x23
#define LSM303_OUT_X_L_A      0x28
#define LSM303_WHO_AM_I_A     0x0F

// Function to initialize I2C
void i2c_initialize() {
    i2c_init(I2C_PORT, 100 * 1000); // 100kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

// Function to write a single register
bool write_register(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACCEL_ADDR, buffer, 2, false);
    return result == 2;
}

// Function to read a single register
bool read_register(uint8_t reg, uint8_t *data) {
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACCEL_ADDR, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(I2C_PORT, LSM303_ACCEL_ADDR, data, 1, false);
    return result == 1;
}

// Function to detect and initialize LSM303DLHC
bool lsm303_init() {
    // Check if LSM303DLHC is present
    uint8_t who_am_i;
    if (!read_register(LSM303_WHO_AM_I_A, &who_am_i)) {
        printf("Error: Cannot communicate with LSM303DLHC\n");
        return false;
    }
    
    if (who_am_i != 0x33) {
        printf("Error: LSM303DLHC not found (WHO_AM_I = 0x%02X)\n", who_am_i);
        return false;
    }
    
    // Enable accelerometer: 50Hz, normal power mode, all axes enabled
    if (!write_register(LSM303_CTRL_REG1_A, 0x47)) {
        return false;
    }
    sleep_ms(10);

    // Configure accelerometer: ±2g range, high resolution mode
    if (!write_register(LSM303_CTRL_REG4_A, 0x08)) {
        return false;
    }
    sleep_ms(10);
    
    return true;
}

// Function to read raw accelerometer data
bool read_accelerometer(int16_t accel_data[3]) {
    uint8_t buffer[6];
    uint8_t reg = LSM303_OUT_X_L_A | 0x80; // Auto-increment bit for multi-byte read
    
    // Set register address and read 6 bytes
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACCEL_ADDR, &reg, 1, true);
    if (result != 1) return false;
    
    result = i2c_read_blocking(I2C_PORT, LSM303_ACCEL_ADDR, buffer, 6, false);
    if (result != 6) return false;

    // LSM303DLHC uses little-endian format (low byte first)
    accel_data[0] = (int16_t)((buffer[1] << 8) | buffer[0]); // X
    accel_data[1] = (int16_t)((buffer[3] << 8) | buffer[2]); // Y
    accel_data[2] = (int16_t)((buffer[5] << 8) | buffer[4]); // Z
    
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB connection
    
    printf("LSM303DLHC Accelerometer Driver\n");
    printf("===============================\n");
    
    // Initialize I2C
    i2c_initialize();
    
    // Initialize LSM303DLHC
    if (!lsm303_init()) {
        printf("Failed to initialize LSM303DLHC!\n");
        while (1) {
            sleep_ms(1000);
        }
    }
    
    printf("LSM303DLHC initialized successfully!\n");
    printf("Reading raw accelerometer data...\n\n");

    uint32_t count = 0;
    while (true) {
        int16_t accel_raw[3];
        
        if (read_accelerometer(accel_raw)) {
            printf("[%lu] X=%6d Y=%6d Z=%6d\n", 
                count++, accel_raw[0], accel_raw[1], accel_raw[2]);
        } else {
            printf("Error reading accelerometer data\n");
        }
        
        sleep_ms(100); // 10Hz update rate
    }

    return 0;
}
