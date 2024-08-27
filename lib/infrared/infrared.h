/**
 * @file infrared.h
 * @brief Declaration of the class responsible for taking readings from the
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

#ifndef INFRARED_H
#define INFRARED_H

#include <Arduino.h>

/**
 * @brief Responsible for reading valued from the GP2Y0E02B Infrared sensor,
 * and controlling the PCA9546 I2C multiplexer that connect the sensor to the
 * bus.
 *
 * Sensor data sheet -
 * https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0e02_03_appl_e.pdf
 *
 * Multiplexer data sheet - https://www.nxp.com/docs/en/data-sheet/PCA9546A.pdf
 */
class Infrared {
   public:
    /**
     * @brief Construct a new Infrared object from its given index on the
     * multiplexer's bus and the distance from the robots centre.
     *
     * @param index The index of the sensor on the multiplexer.
     */
    Infrared(uint8_t index);

    /**
     * @brief Sets up the sensor by setting up the I2C communication.
     */
    void setup();

    /**
     * @brief Read the distance from the sensor.
     *
     * @return (int16_t) The distance read from the sensor, in millimeters.
     * A return value of -1 indicates a reading error, typically reading exceeds
     * the max value.
     */
    int16_t read();

   private:
    /**
     * @brief The index of this sensor on the Multiplexer's bus.
     */
    uint8_t _index;

    /**
     * @brief The shift value of the infrared sensor.
     */
    uint8_t _shiftValue;

    /**
     * @brief Connects this sensor the the I2C bus by writing its index to the
     * multiplexer.
     */
    void _setMultiplexer();
};

#endif  // INFRARED_H