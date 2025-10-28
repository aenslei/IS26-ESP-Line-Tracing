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

// Filter instances
static lpf_state_t lpf_accel = {.initialized = false};
static kalman_state_t kalman_accel = {.Q = 0.1f, .R = 10.0f, .initialized = false};
static lpf_state_t lpf_mag = {.initialized = false};
static kalman_state_t kalman_mag = {.Q = 0.05f, .R = 5.0f, .initialized = false};

// Function declarations
bool write_register(uint8_t reg, uint8_t data);
bool write_mag_register(uint8_t reg, uint8_t data);

// Remove the alias - we'll rename the function directly

// Function declarations
bool mag_init(void);

// Function to initialize I2C
void i2c_initialize() {
    i2c_init(I2C_PORT, 100 * 1000); // set i2c speed to 100khz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); // gp0 as sda pin
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C); // gp1 as scl pin
    gpio_pull_up(I2C_SDA_PIN); // enable pullup on sda
    gpio_pull_up(I2C_SCL_PIN); // enable pullup on scl
}

// Function to write a single register
bool write_register(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data}; // pack reg addr & data together
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACCEL_ADDR, buffer, 2, false); // send both bytes
    return result == 2; // check if 2 bytes sent ok
}

// Function to read a single register
bool read_register(uint8_t reg, uint8_t *data) {
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACCEL_ADDR, &reg, 1, true); // tell sensor which reg to read
    if (result != 1) return false; // check reg addr sent ok
    result = i2c_read_blocking(I2C_PORT, LSM303_ACCEL_ADDR, data, 1, false); // read the data byte
    return result == 1; // check if data received ok
}

// Function to detect and initialize LSM303DLHC
// Function to initialize LSM303DLHC (both accelerometer and magnetometer)  
bool lsm303_init() {
    // Check if LSM303DLHC is present
    uint8_t who_am_i;
    if (!read_register(LSM303_WHO_AM_I_A, &who_am_i)) { // try to read sensor id
        printf("Error: Cannot communicate with LSM303DLHC\n");
        return false;
    }
    
    if (who_am_i != 0x33) { // check if correct sensor id (0x33)
        printf("Error: LSM303DLHC not found (WHO_AM_I = 0x%02X)\n", who_am_i);
        return false;
    }
    
    // Enable accelerometer: 50Hz, normal power mode, all axes enabled
    if (!write_register(LSM303_CTRL_REG1_A, 0x47)) { // 0x47 = 50hz + normal mode + xyz on
        return false;
    }
    sleep_ms(10); // wait for setting to apply

    // Configure accelerometer: ±2g range, high resolution mode
    if (!write_register(LSM303_CTRL_REG4_A, 0x08)) { // 0x08 = 2g range + high res
        return false;
    }
    sleep_ms(10); // wait for setting to apply
    
    // Initialize magnetometer
    if (!mag_init()) { // setup mag part of chip
        printf("Warning: Magnetometer initialization failed\n");
    }
    
    return true;
}

// Function to initialize magnetometer
bool mag_init(void) {
    // Configure magnetometer: 15Hz output rate
    if (!write_mag_register(LSM303_CRA_REG_M, 0x10)) { // 0x10 = 15hz data rate
        return false;
    }
    sleep_ms(10); // wait for mag setting
    
    // Set gain to ±1.3 gauss
    if (!write_mag_register(LSM303_CRB_REG_M, 0x20)) { // 0x20 = ±1.3 gauss range
        return false;
    }
    sleep_ms(10); // wait for gain setting
    
    // Enable continuous conversion mode
    if (!write_mag_register(LSM303_MR_REG_M, 0x00)) { // 0x00 = continuous mode (always measuring)
        return false;
    }
    sleep_ms(10); // wait for mode setting
    
    return true;
}

// Function to write magnetometer register
bool write_mag_register(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data}; // pack reg addr & data for mag
    int result = i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, buffer, 2, false); // send to mag addr (0x1e)
    return result == 2; // check if both bytes sent ok
}

// Function to read magnetometer register
bool read_mag_register(uint8_t reg, uint8_t *data) {
    int result = i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, &reg, 1, true); // tell mag which reg to read
    if (result != 1) return false; // check reg addr sent ok
    result = i2c_read_blocking(I2C_PORT, LSM303_MAG_ADDR, data, 1, false); // read data from mag
    return result == 1; // check if mag data received ok
}

// Function to read raw accelerometer data
bool read_accelerometer(int16_t accel_data[3]) {
    uint8_t buffer[6];
    uint8_t reg = LSM303_OUT_X_L_A | 0x80; // 0x80 = auto-increment to read all xyz in one go
    
    // Set register address and read 6 bytes
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACCEL_ADDR, &reg, 1, true); // tell accel where to start reading
    if (result != 1) return false; // check reg addr sent ok
    
    result = i2c_read_blocking(I2C_PORT, LSM303_ACCEL_ADDR, buffer, 6, false); // read 6 bytes (xyz = 2 bytes each)
    if (result != 6) return false; // check all 6 bytes received

    // LSM303DLHC uses little-endian format (low byte first)
    accel_data[0] = (int16_t)((buffer[1] << 8) | buffer[0]); // x = high byte + low byte
    accel_data[1] = (int16_t)((buffer[3] << 8) | buffer[2]); // y = high byte + low byte  
    accel_data[2] = (int16_t)((buffer[5] << 8) | buffer[4]); // z = high byte + low byte
    
    return true;
}

// Function to read raw magnetometer data
bool read_magnetometer(int16_t mag_data[3]) {
    uint8_t buffer[6];
    uint8_t reg = LSM303_OUT_X_H_M; // start at x high byte reg
    
    // Set register address and read 6 bytes
    int result = i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, &reg, 1, true); // tell mag where to start reading
    if (result != 1) return false; // check reg addr sent ok
    
    result = i2c_read_blocking(I2C_PORT, LSM303_MAG_ADDR, buffer, 6, false); // read 6 bytes from mag
    if (result != 6) return false; // check all 6 bytes received

    // LSM303DLHC magnetometer uses big-endian format (high byte first)
    // Order is X, Z, Y (note: Z and Y are swapped in register layout)
    mag_data[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // x = high + low byte (big endian)
    mag_data[1] = (int16_t)((buffer[4] << 8) | buffer[5]); // y = high + low byte (from pos 4,5)
    mag_data[2] = (int16_t)((buffer[2] << 8) | buffer[3]); // z = high + low byte (from pos 2,3)
    
    return true;
}

// Function to determine compass quadrant from magnetometer data
int get_quadrant(float mag_x, float mag_y) {
    if (mag_x > 0 && mag_y > 0) return 1; // northeast (0-90°)
    if (mag_x < 0 && mag_y > 0) return 2; // northwest (90-180°)  
    if (mag_x < 0 && mag_y < 0) return 3; // southwest (180-270°)
    if (mag_x > 0 && mag_y < 0) return 4; // southeast (270-360°)
    return 0; // edge case (exactly on axis)
}

// Debug function to show detailed magnetometer analysis
void debug_magnetometer(float mag_x, float mag_y, float heading) {
    printf("    DEBUG: mag_x=%8.1f, mag_y=%8.1f", mag_x, mag_y);
    printf(" | atan2f result=%7.3f rad = %6.1f deg", atan2f(mag_y, mag_x), atan2f(mag_y, mag_x) * 180.0f / 3.14159265f);
    printf(" | final heading=%6.1f°\n", heading);
}

// Function to calculate 3D headings from magnetometer data (0-360 degrees)
typedef struct {
    float xy_heading;  // horizontal compass (x,y plane) - traditional compass
    float xz_heading;  // vertical side tilt (x,z plane) - pitch-relative
    float yz_heading;  // vertical front tilt (y,z plane) - roll-relative
} heading_3d_t;

// Calculate traditional horizontal compass heading (XY plane)
float calculate_heading_xy(float mag_x, float mag_y) {
    float heading_rad = atan2f(mag_y, mag_x); // horizontal compass angle
    float heading = heading_rad * 180.0f / 3.14159265f; // rad to deg
    if (heading < 0) heading += 360.0f; // convert to 0-360 range
    return heading;
}

// Calculate vertical heading in XZ plane (side tilt)
float calculate_heading_xz(float mag_x, float mag_z) {
    float heading_rad = atan2f(mag_z, mag_x); // vertical side angle
    float heading = heading_rad * 180.0f / 3.14159265f; // rad to deg
    if (heading < 0) heading += 360.0f; // convert to 0-360 range
    return heading;
}

// Calculate vertical heading in YZ plane (front tilt)  
float calculate_heading_yz(float mag_y, float mag_z) {
    float heading_rad = atan2f(mag_z, mag_y); // vertical front angle
    float heading = heading_rad * 180.0f / 3.14159265f; // rad to deg
    if (heading < 0) heading += 360.0f; // convert to 0-360 range
    return heading;
}

// Calculate all 3 heading angles at once
heading_3d_t calculate_headings_3d(float mag_x, float mag_y, float mag_z) {
    heading_3d_t headings;
    headings.xy_heading = calculate_heading_xy(mag_x, mag_y); // horizontal compass
    headings.xz_heading = calculate_heading_xz(mag_x, mag_z); // side tilt
    headings.yz_heading = calculate_heading_yz(mag_y, mag_z); // front tilt
    return headings;
}

// Legacy function for backward compatibility
float calculate_heading(float mag_x, float mag_y) {
    return calculate_heading_xy(mag_x, mag_y); // just return horizontal heading
}

// ========== FILTER IMPLEMENTATIONS ==========

// Low-Pass Filter Implementation
// Formula: y[n] = α * x[n] + (1-α) * y[n-1]
// Where: α = LPF_ALPHA, x[n] = current input, y[n-1] = previous output
void apply_low_pass_filter(int16_t raw_data[3], float filtered_data[3]) {
    for (int i = 0; i < 3; i++) { // do xyz axes
        if (!lpf_accel.initialized) {
            // First sample: initialize with raw data
            lpf_accel.prev_output[i] = (float)raw_data[i]; // first time - just use raw value
            lpf_accel.initialized = true; // mark as ready
        } else {
            // Apply low-pass filter formula
            lpf_accel.prev_output[i] = LPF_ALPHA * (float)raw_data[i] +  // new * 0.2 
                                      (1.0f - LPF_ALPHA) * lpf_accel.prev_output[i]; // + old * 0.8
        }
        filtered_data[i] = lpf_accel.prev_output[i]; // copy result out
    }
}

// Simple 1D Kalman Filter Implementation
// State: acceleration value
// Prediction: x_pred = x_prev (assuming constant acceleration)
// Update: x_new = x_pred + K * (measurement - x_pred)
// Kalman Gain: K = P / (P + R)
void apply_kalman_filter(float lpf_data[3], float kalman_data[3]) {
    for (int i = 0; i < 3; i++) { // do xyz axes
        if (!kalman_accel.initialized) {
            // Initialize Kalman filter with first measurement
            kalman_accel.x[i] = lpf_data[i]; // first guess = input
            kalman_accel.P[i] = 1.0f; // error estimate
            kalman_accel.initialized = true; // mark ready
        } else {
            // Prediction Step
            // x_pred = x_prev (no control input)
            float x_pred = kalman_accel.x[i]; // keep same prediction
            float P_pred = kalman_accel.P[i] + kalman_accel.Q; // add process noise
            
            // Update Step
            float K = P_pred / (P_pred + kalman_accel.R);  // kalman gain = how much to trust new data
            kalman_accel.x[i] = x_pred + K * (lpf_data[i] - x_pred); // blend old + new
            kalman_accel.P[i] = (1.0f - K) * P_pred; // update error estimate
        }
        kalman_data[i] = kalman_accel.x[i]; // copy result
    }
}

void apply_mag_low_pass_filter(float raw_data[3], float filtered_data[3]) {
    for (int i = 0; i < 3; i++) { // do xyz axes
        // Initialize with first measurement if not already initialized
        if (!lpf_mag.initialized) {
            lpf_mag.prev_output[i] = raw_data[i]; // first time - use raw value
            lpf_mag.initialized = true; // mark ready
        }
        
        // Apply low-pass filter using global LPF_ALPHA constant
        filtered_data[i] = LPF_ALPHA * raw_data[i] + (1.0f - LPF_ALPHA) * lpf_mag.prev_output[i]; // new*0.2 + old*0.8
        lpf_mag.prev_output[i] = filtered_data[i]; // save for next time
    }
}

void apply_mag_kalman_filter(float lpf_data[3], float kalman_data[3]) {
    for (int i = 0; i < 3; i++) { // do xyz axes
        if (!kalman_mag.initialized) {
            // Initialize Kalman filter with first measurement
            kalman_mag.x[i] = lpf_data[i]; // first guess = input
            kalman_mag.P[i] = 1.0f; // error estimate
            kalman_mag.initialized = true; // mark ready
        } else {
            // Prediction Step
            // x_pred = x_prev (no control input)
            float x_pred = kalman_mag.x[i]; // keep same prediction
            float P_pred = kalman_mag.P[i] + kalman_mag.Q; // add process noise
            
            // Update Step
            float K = P_pred / (P_pred + kalman_mag.R);  // kalman gain = how much to trust new data
            kalman_mag.x[i] = x_pred + K * (lpf_data[i] - x_pred); // blend old + new
            kalman_mag.P[i] = (1.0f - K) * P_pred; // update error estimate
        }
        kalman_data[i] = kalman_mag.x[i]; // copy result
    }
}

// Convert raw data to g-force (±2g range: 1000 LSB/g for LSM303DLHC)
void convert_to_g(float filtered_data[3], float g_data[3]) {
    for (int i = 0; i < 3; i++) { // do xyz axes
        g_data[i] = filtered_data[i] / 1000.0f; // raw counts to g-force (lsm303 uses 1000 counts per g)
    }
}

/*
// Standalone main function - commented out when used as library
int main() {
    stdio_init_all(); // start usb serial
    sleep_ms(2000); // wait for usb connection
    
    printf("LSM303DLHC Accelerometer Driver\n");
    printf("===============================\n");
    
    // Initialize I2C
    i2c_initialize(); // setup i2c pins + pullups
    
    // Initialize LSM303DLHC
    if (!lsm303_init()) { // try to wake up sensor
        printf("Failed to initialize LSM303DLHC!\n");
        while (1) { // stuck here if sensor not found
            sleep_ms(1000);
        }
    }
    
    printf("LSM303DLHC initialized successfully!\n");
    printf("Reading filtered accelerometer and magnetometer data...\n");
    printf("Format: [Count] Accel: X=value Y=value Z=value (g) | 3D Headings: XY=° XZ=° YZ=° Q=quadrant\n");
    printf("XY=horizontal compass, XZ=side tilt, YZ=front tilt\n");
    printf("Quadrants: 1=NE(0-90°) 2=NW(90-180°) 3=SW(180-270°) 4=SE(270-360°)\n\n");

    uint32_t count = 0; // reading counter
    while (true) { // main loop
        int16_t accel_raw[3]; // raw accelerometer data
        float accel_lpf[3]; // after low-pass filter
        float accel_kalman[3]; // after kalman filter
        float g_data[3]; // final g-force values
        
        int16_t mag_raw[3]; // raw magnetometer data
        float mag_raw_float[3]; // convert to float for filtering
        float mag_lpf[3]; // after low-pass filter
        float mag_kalman[3]; // after kalman filter
        heading_3d_t headings_3d; // all 3 compass headings
        int quadrant; // which compass quadrant (1-4)
        
        bool accel_ok = read_accelerometer(accel_raw); // try to read accelerometer
        bool mag_ok = read_magnetometer(mag_raw); // try to read magnetometer
        
        if (accel_ok) { // if accelerometer read worked
            // Process accelerometer data
            apply_low_pass_filter(accel_raw, accel_lpf); // smooth out noise
            apply_kalman_filter(accel_lpf, accel_kalman); // more advanced filtering
            convert_to_g(accel_kalman, g_data); // convert counts to g-force
        }
        
        if (mag_ok) { // if magnetometer read worked
            // Process magnetometer data
            // Convert raw magnetometer to float for filtering
            for (int i = 0; i < 3; i++) { // convert xyz to float
                mag_raw_float[i] = (float)mag_raw[i];
            }
            
            apply_mag_low_pass_filter(mag_raw_float, mag_lpf); // smooth out noise
            apply_mag_kalman_filter(mag_lpf, mag_kalman); // more advanced filtering
            
            // Calculate all 3D headings from filtered magnetometer data
            headings_3d = calculate_headings_3d(mag_kalman[0], mag_kalman[1], mag_kalman[2]); // get all 3 compass directions
            quadrant = get_quadrant(mag_kalman[0], mag_kalman[1]); // get compass quadrant (xy plane)
            
            // Debug output every 10th reading to avoid spam
            if (count % 10 == 0) {
                debug_magnetometer(mag_kalman[0], mag_kalman[1], headings_3d.xy_heading);
            }
        }
        
        if (accel_ok && mag_ok) { // both sensors working
            // Display combined results with raw magnetometer values for debugging
            printf("[%lu] Accel: X=%6.2f Y=%6.2f Z=%6.2f | XY=%6.1f° XZ=%6.1f° YZ=%6.1f° Q=%d | Raw Mag: X=%6.0f Y=%6.0f Z=%6.0f\n", 
                count++, g_data[0], g_data[1], g_data[2], 
                headings_3d.xy_heading, headings_3d.xz_heading, headings_3d.yz_heading, quadrant,
                mag_kalman[0], mag_kalman[1], mag_kalman[2]); // show accel + 3d compass + quadrant + raw mag for debugging
        } else if (accel_ok) { // only accel working
            printf("[%lu] Accel: X=%6.2f Y=%6.2f Z=%6.2f | Mag: ERROR\n", 
                count++, g_data[0], g_data[1], g_data[2]); // show accel only
        } else if (mag_ok) { // only mag working
            printf("[%lu] Accel: ERROR | XY=%6.1f° XZ=%6.1f° YZ=%6.1f° Q=%d | Raw Mag: X=%6.0f Y=%6.0f Z=%6.0f\n", 
                count++, headings_3d.xy_heading, headings_3d.xz_heading, headings_3d.yz_heading, quadrant,
                mag_kalman[0], mag_kalman[1], mag_kalman[2]); // show 3d compass + quadrant only + raw mag
        } else { // both broken
            printf("[%lu] ERROR: Both sensors failed\n", count++); // show error
        }
        
        sleep_ms(100); // wait 100ms = 10hz update rate
    }

    return 0; // never reached
}
*/

