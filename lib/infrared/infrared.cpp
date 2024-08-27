/**
 * @file infrared.cpp
 * @brief Definitions of the class responsible for taking readings from the
 * GP2Y0E02B Infrared sensor, while utilizing the PCA9546 multiplexer.
 *
 * The data sheet for the GP2Y0E02B IR sensor can be found here,
 * https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0e02_03_appl_e.pdf.
 *
 * The data sheet for the PCA9546 Multiplexer can be found here,
 * https://www.nxp.com/docs/en/data-sheet/PCA9546A.pdf.
 *
 * @author Harry Boyd - https://github.com/HBoyd255
 * @date 2024-08-13
 * @copyright Copyright (c) 2024
 */
#include "infrared.h"

#include <Wire.h>

// Multiplexer
// https://www.nxp.com/docs/en/data-sheet/PCA9546A.pdf
// Multiplexer address, shifted to provide 7-bit version
#define MULTIPLEXER_SLAVE_ADDRESS 0xE0 >> 1
#define MULTIPLEXER_CHANNEL_COUNT 4

// Infrared sensor
// Address for IR sensor, shifted to provide 7-bit version
#define IR_SLAVE_ADDRESS 0x80 >> 1

// https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0e02_03_appl_e.pdf#page=16
// Address  | Register Name  | Reg Field    | Default   | R/W   | Description
// 0x35     | Shift Bit      | [2:0]        | 0x02      | R/W   |  0x01 =
// MaximumDisplay 128cm  0x02=Maximum Display 64cm
#define IR_SHIFT_REG_ADDRESS 0x35

// 0x5E     |  Distance[11:4] | [7:0]       | -         | R     |
// 0x5F     |  Distance[ 3:0] | [3:0]       | -         | R     |
// Distance Value =(Distance[11:4]*16+Distance[3:0])/16/2^n n : Shift Bit
// (Register 0x35)
#define IR_DISTANCE_REG_ADDRESS 0x5E

#define MAX_IR_RANGE 639

/**
 * @brief Construct a new Infrared object from its given index on the
 * multiplexer's bus and the distance from the robots centre.
 *
 * @param index The index of the sensor on the multiplexer.
 * @param distanceFromCentre The distance in millimeters that this sensor is
 * from the centre of the robot.
 */
Infrared::Infrared(uint8_t index) : _index(index) {}

/**
 * @brief Sets up the sensor by setting up the I2C communication.
 */
void Infrared::setup() {
    // Initialise I2C communication.
    Wire.begin();

    // Connect the current sensor to to the I2C bus.
    this->_setMultiplexer();

    // Start the I2C transmission to the slave address.
    Wire.beginTransmission(IR_SLAVE_ADDRESS);

    // Set the sensor to 64cm mode.
    Wire.write(IR_SHIFT_REG_ADDRESS);

    // End the transmission.
    Wire.endTransmission();

    // Request 1 byte from the slave address.
    Wire.requestFrom(IR_SLAVE_ADDRESS, 1);

    // If I2C communication is not available, raise a critical error.
    if (!Wire.available()) {
        Serial.println("Wire not available");
    }

    // Read in the shift value from the I2C bus.
    this->_shiftValue = Wire.read();
}

/**
 * @brief Read the distance from the sensor.
 *
 * @return (int16_t) The distance read from the sensor, in millimeters.
 * A return value of -1 indicates a reading error, typically reading exceeds
 * the max value.
 */
int16_t Infrared::read() {
    // Connect the current sensor to to the I2C bus.
    this->_setMultiplexer();

    // Start the I2C transmission to the slave address.
    Wire.beginTransmission(IR_SLAVE_ADDRESS);

    // Write the distance register address to the sensor
    Wire.write(IR_DISTANCE_REG_ADDRESS);

    // End the transmission.
    Wire.endTransmission();

    // Request 2 bytes from the sensor.
    Wire.requestFrom(IR_SLAVE_ADDRESS, 2);

    // If data cannot be read from the sensor, raise a critical error.
    if (Wire.available() < 2) {
        Serial.println("Wire not available");
        return -1;
    }

    // read the first byte in as the most significant part
    uint8_t high = Wire.read();
    //  read the second byte in as the least significant part
    uint8_t low = Wire.read();

    // The given formula for calculating distance is:
    // distance(cm) = (high * 16 + low)/16/(int)pow(2,shift);
    // Which can be simplified to:
    // distance(mm) = (((high << 4) | low) * 10) >> (4 + shiftValue);

    uint16_t distance = (((high << 4) | low) * 10) >> (4 + this->_shiftValue);

    // if the distance is out of range, return -1.
    return (distance < MAX_IR_RANGE) ? distance : -1;
}

/**
 * @brief Connects this sensor the the I2C bus by writing its index to the
 * multiplexer.
 */
void Infrared::_setMultiplexer() {
    // Connect to the multiplexer via I2C.
    Wire.beginTransmission(MULTIPLEXER_SLAVE_ADDRESS);
    // Write the index of this sensor to the multiplexer.
    Wire.write(1 << this->_index);
    // End the transmission.
    Wire.endTransmission();
}
