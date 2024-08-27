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

#include "pixels.h"
#include "systemInfo.h"
#include "ultrasonic.h"

Pixels pixels(PIXELS_DATA_PIN, LED_COUNT);

Ultrasonic ultrasonic(ULTRASONIC_TRIGGER, ULTRASONIC_ECHO,
                      ULTRASONIC_TIMEOUT_MICROSECONDS, ULTRASONIC_MAX_DISTANCE,
                      ULTRASONIC_DATA_SHELF_LIFE);
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

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

    pixels.setPixel(7, Colour(0, 0, ultrasonicBrightness), true);
    pixels.setPixel(8, Colour(0, 0, ultrasonicBrightness), true);
}
