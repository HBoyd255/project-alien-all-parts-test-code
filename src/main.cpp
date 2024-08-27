/**
 * @file main.cpp
 * @brief The main file and entry point of the project.
 *
 * @author Harry Boyd - https://github.com/HBoyd255
 * @date 2024-08-27
 * @copyright Copyright (c) 2024
 */

//  ██████╗ ██████╗  ██████╗      ██╗███████╗ ██████╗████████╗
//  ██╔══██╗██╔══██╗██╔═══██╗     ██║██╔════╝██╔════╝╚══██╔══╝
//  ██████╔╝██████╔╝██║   ██║     ██║█████╗  ██║        ██║
//  ██╔═══╝ ██╔══██╗██║   ██║██   ██║██╔══╝  ██║        ██║
//  ██║     ██║  ██║╚██████╔╝╚█████╔╝███████╗╚██████╗   ██║
//  ╚═╝     ╚═╝  ╚═╝ ╚═════╝  ╚════╝ ╚══════╝ ╚═════╝   ╚═╝
//
//   █████╗    ██╗        ██╗   ███████╗   ███╗   ██╗
//  ██╔══██╗   ██║        ██║   ██╔════╝   ████╗  ██║
//  ███████║   ██║        ██║   █████╗     ██╔██╗ ██║
//  ██╔══██║   ██║        ██║   ██╔══╝     ██║╚██╗██║
//  ██║  ██║██╗███████╗██╗██║██╗███████╗██╗██║ ╚████║██╗
//  ╚═╝  ╚═╝╚═╝╚══════╝╚═╝╚═╝╚═╝╚══════╝╚═╝╚═╝  ╚═══╝╚═╝

// Ascii text generated at https://patorjk.com/software/taag/
// Font used - ANSI Shadow

#include <Arduino.h>

#include "binary.h"
#include "bumper.h"
#include "pixels.h"
#include "systemInfo.h"
#include "ultrasonic.h"

Bumper bumper(BUMPER_SHIFT_REG_DATA, COMMON_SHIFT_REG_LOAD,
              COMMON_SHIFT_REG_CLOCK, BUMPER_BIT_OFFSET);

Pixels pixels(PIXELS_DATA_PIN, LED_COUNT);

Ultrasonic ultrasonic(ULTRASONIC_TRIGGER, ULTRASONIC_ECHO,
                      ULTRASONIC_TIMEOUT_MICROSECONDS, ULTRASONIC_MAX_DISTANCE,
                      ULTRASONIC_DATA_SHELF_LIFE);
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    bumper.setup();

    pixels.setup();

    ultrasonic.setup([]() { ultrasonic.isr(); });
}

void loop() {
    ultrasonic.poll();
    delay(10);

    int ultrasonicDistance = ultrasonic.read();

    Serial.print(" Ultrasonic reading in millimetres:");
    Serial.println(ultrasonicDistance);

    // Normalise the distance to brightness between 0 and 255

    // If the distance can not be read (Reading of -1) then set the distance to
    // an arbitrary high value.
    if (ultrasonicDistance < 1) {
        ultrasonicDistance = 1000;
    }

    int ultrasonicBrightness = map(ultrasonicDistance, 50, 300, 255, 0);

    // Clamp the brightness to between 0 and 255.
    if (ultrasonicBrightness > 255) {
        ultrasonicBrightness = 255;
    }
    if (ultrasonicBrightness < 0) {
        ultrasonicBrightness = 0;
    }

    uint8_t bumperState = bumper.read();

    Serial.print(" Bumper state:");
    printByte(bumperState, ",");

    bool bumperF = bool(bumperState & 0b00000001);
    bool bumperFR = bool(bumperState & 0b00000010);
    bool bumperR = bool(bumperState & 0b00000100);
    bool bumperBR = bool(bumperState & 0b00001000);
    bool bumperB = bool(bumperState & 0b00010000);
    bool bumperBL = bool(bumperState & 0b00100000);
    bool bumperL = bool(bumperState & 0b01000000);
    bool bumperFL = bool(bumperState & 0b10000000);

    pixels.setPixel(0, Colour(0, bumperB * 255, 0), true);
    pixels.setPixel(1, Colour(0, bumperBL * 255, 0), true);
    pixels.setPixel(2, Colour(0, bumperBL * 255, 0), true);
    pixels.setPixel(3, Colour(0, bumperL * 255, 0), true);
    pixels.setPixel(4, Colour(0, bumperL * 255, 0), true);
    pixels.setPixel(5, Colour(0, bumperFL * 255, 0), true);
    pixels.setPixel(6, Colour(0, bumperFL * 255, 0), true);
    pixels.setPixel(7, Colour(0, bumperF * 255, ultrasonicBrightness), true);
    pixels.setPixel(8, Colour(0, bumperF * 255, ultrasonicBrightness), true);
    pixels.setPixel(9, Colour(0, bumperFR * 255, 0), true);
    pixels.setPixel(10, Colour(0, bumperFR * 255, 0), true);
    pixels.setPixel(11, Colour(0, bumperR * 255, 0), true);
    pixels.setPixel(12, Colour(0, bumperR * 255, 0), true);
    pixels.setPixel(13, Colour(0, bumperBR * 255, 0), true);
    pixels.setPixel(14, Colour(0, bumperBR * 255, 0), true);
    pixels.setPixel(15, Colour(0, bumperB * 255, 0), true);
}
