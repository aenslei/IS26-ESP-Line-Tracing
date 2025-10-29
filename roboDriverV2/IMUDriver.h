#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C Configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0  // GP0 (SDA)
#define I2C_SCL_PIN 1  // GP1 (SCL)
#define LSM303_ACCEL_ADDR 0x19   // LSM303DLHC accelerometer I2C address
#define LSM303_MAG_ADDR   0x1E   // LSM303DLHC magnetometer I2C address

// LSM303DLHC Accelerometer Registers
#define LSM303_CTRL_REG1_A    0x20
#define LSM303_CTRL_REG4_A    0x23
#define LSM303_OUT_X_L_A      0x28
#define LSM303_WHO_AM_I_A     0x0F

// LSM303DLHC Magnetometer Registers
#define LSM303_CRA_REG_M      0x00
#define LSM303_CRB_REG_M      0x01
#define LSM303_MR_REG_M       0x02
#define LSM303_OUT_X_H_M      0x03

// Filter Configuration
#define FILTER_SAMPLES 10        // Number of samples for moving average
#define LPF_ALPHA 0.2f          // Low-pass filter coefficient (0-1, lower = more smoothing)

// Low-Pass Filter Structure
typedef struct {
    float prev_output[3];       // Previous filtered output [X, Y, Z]
    bool initialized;           // First-run flag
} lpf_state_t;

// Simple Kalman Filter Structure (1D, applied to each axis)
typedef struct {
    float x[3];                 // State estimate [X, Y, Z]
    float P[3];                 // Estimation error covariance [X, Y, Z]
    float Q;                    // Process noise covariance
    float R;                    // Measurement noise covariance
    bool initialized;           // First-run flag
} kalman_state_t;

// 3D Heading Structure
typedef struct {
    float xy_heading;  // horizontal compass (x,y plane) - traditional compass
    float xz_heading;  // vertical side tilt (x,z plane) - pitch-relative
    float yz_heading;  // vertical front tilt (y,z plane) - roll-relative
} heading_3d_t;

// Function Declarations

// === I2C Communication Functions ===
void i2c_initialize(void);
bool write_register(uint8_t reg, uint8_t data);
bool read_register(uint8_t reg, uint8_t *data);
bool write_mag_register(uint8_t reg, uint8_t data);
bool read_mag_register(uint8_t reg, uint8_t *data);

// === Sensor Initialization Functions ===
bool lsm303_init(void);
bool mag_init(void);

// === Data Reading Functions ===
bool read_accelerometer(int16_t accel_data[3]);
bool read_magnetometer(int16_t mag_data[3]);

// === Compass and Quadrant Functions ===
int get_quadrant(float mag_x, float mag_y);
float calculate_heading_xy(float mag_x, float mag_y);
float calculate_heading_xz(float mag_x, float mag_z);
float calculate_heading_yz(float mag_y, float mag_z);
heading_3d_t calculate_headings_3d(float mag_x, float mag_y, float mag_z);
float calculate_heading(float mag_x, float mag_y); // Legacy function

// === Filter Functions ===
void apply_low_pass_filter(int16_t raw_data[3], float filtered_data[3]);
void apply_kalman_filter(float lpf_data[3], float kalman_data[3]);
void apply_mag_low_pass_filter(float raw_data[3], float filtered_data[3]);
void apply_mag_kalman_filter(float lpf_data[3], float kalman_data[3]);

// === Utility Functions ===
void convert_to_g(float filtered_data[3], float g_data[3]);
void debug_magnetometer(float mag_x, float mag_y, float heading);

// === High-Level API Functions for Easy Integration ===

/**
 * @brief Initialize the complete IMU system
 * @return true if initialization successful, false otherwise
 */
bool imu_init(void);

/**
 * @brief Read and process all IMU data in one call
 * @param accel_g Output: accelerometer data in g-force [x,y,z]
 * @param headings_3d Output: all 3D compass headings
 * @param quadrant Output: compass quadrant (1-4)
 * @return true if successful, false if sensor read failed
 */
bool imu_read_all_data(float accel_g[3], heading_3d_t *headings_3d, int *quadrant);

/**
 * @brief Get just the horizontal compass heading (most common use)
 * @return Heading in degrees (0-360), or -1 if read failed
 */
float imu_get_compass_heading(void);

/**
 * @brief Get accelerometer tilt angles
 * @param roll Output: roll angle in degrees
 * @param pitch Output: pitch angle in degrees  
 * @return true if successful, false if read failed
 */
bool imu_get_tilt_angles(float *roll, float *pitch);

#endif // IMU_DRIVER_H
