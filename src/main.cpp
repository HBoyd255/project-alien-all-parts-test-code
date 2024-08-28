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
#include <NESControllerInterface.h>

#include "binary.h"
#include "bumper.h"
#include "infrared.h"
#include "pixels.h"
#include "systemInfo.h"
#include "ultrasonic.h"

NESControllerInterface nesController(NES_SHIFT_REG_DATA, COMMON_SHIFT_REG_LOAD,
                                     COMMON_SHIFT_REG_CLOCK);

Bumper bumper(BUMPER_SHIFT_REG_DATA, COMMON_SHIFT_REG_LOAD,
              COMMON_SHIFT_REG_CLOCK, BUMPER_BIT_OFFSET);

Pixels pixels(PIXELS_DATA_PIN, LED_COUNT);

Ultrasonic ultrasonic(ULTRASONIC_TRIGGER, ULTRASONIC_ECHO,
                      ULTRASONIC_TIMEOUT_MICROSECONDS, ULTRASONIC_MAX_DISTANCE,
                      ULTRASONIC_DATA_SHELF_LIFE);

Infrared infraredLeft = Infrared(LEFT_INFRARED_INDEX);
Infrared infraredFrontLeft = Infrared(FRONT_LEFT_INFRARED_INDEX);
Infrared infraredFrontRight = Infrared(FRONT_RIGHT_INFRARED_INDEX);
Infrared infraredRight = Infrared(RIGHT_INFRARED_INDEX);

int leftMotorSteps = 0;
int rightMotorSteps = 0;

void leftMotorISR() {
    if (digitalRead(LEFT_MOTOR_ENCODER_A_PIN) !=
        digitalRead(LEFT_MOTOR_ENCODER_B_PIN)) {
        leftMotorSteps++;
    } else {
        leftMotorSteps--;
    }
}

void rightMotorISR() {
    if (digitalRead(RIGHT_MOTOR_ENCODER_A_PIN) !=
        digitalRead(RIGHT_MOTOR_ENCODER_B_PIN)) {
        rightMotorSteps++;
    } else {
        rightMotorSteps--;
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    pixels.setup();

    ultrasonic.setup([]() { ultrasonic.isr(); });

    infraredLeft.setup();
    infraredFrontLeft.setup();
    infraredFrontRight.setup();
    infraredRight.setup();

    // Setup left motor encoders.
    pinMode(LEFT_MOTOR_ENCODER_A_PIN, INPUT);
    pinMode(LEFT_MOTOR_ENCODER_B_PIN, INPUT);

    // Setup left motor drive.
    pinMode(LEFT_MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);

    // Setup right motor encoders.
    pinMode(RIGHT_MOTOR_ENCODER_A_PIN, INPUT);
    pinMode(RIGHT_MOTOR_ENCODER_B_PIN, INPUT);

    // Setup right motor drive.
    pinMode(RIGHT_MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);

    // Attach the interrupt service routines.
    attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_ENCODER_A_PIN),
                    leftMotorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_ENCODER_A_PIN),
                    rightMotorISR, CHANGE);
}

int distanceToBrightness(int distance) {
    // If the distance can not be read (Reading of -1) then set the distance to
    // an arbitrary high value.
    if (distance < 1) {
        distance = 1000;
    }

    int brightness = map(distance, 50, 300, 255, 0);
    if (brightness > 255) {
        brightness = 255;
    }
    if (brightness < 0) {
        brightness = 0;
    }
    return brightness;
}

void loop() {
    ultrasonic.poll();
    delay(10);

    int ultrasonicDistance = ultrasonic.read();

    Serial.print(" Ultrasonic reading in millimetres:");
    Serial.println(ultrasonicDistance);

    // Normalise the distance to brightness between 0 and 255

    int ultrasonicBrightness = distanceToBrightness(ultrasonicDistance);

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

    int leftIRDistance = infraredLeft.read();
    int frontLeftIRDistance = infraredFrontLeft.read();
    int frontRightIRDistance = infraredFrontRight.read();
    int rightIRDistance = infraredRight.read();

    // Normalise the distance to brightness between 0 and 255
    int leftIRBrightness = distanceToBrightness(leftIRDistance);
    int frontLeftIRBrightness = distanceToBrightness(frontLeftIRDistance);
    int frontRightIRBrightness = distanceToBrightness(frontRightIRDistance);
    int rightIRBrightness = distanceToBrightness(rightIRDistance);

    Serial.print(" Left infrared reading in millimetres:");
    Serial.println(leftIRDistance);

    Serial.print(" Front left infrared reading in millimetres:");
    Serial.println(frontLeftIRDistance);

    Serial.print(" Front right infrared reading in millimetres:");
    Serial.println(frontRightIRDistance);

    Serial.print(" Right infrared reading in millimetres:");
    Serial.println(rightIRDistance);

    pixels.setPixel(0, Colour(0, bumperB * 255, 0), true);
    pixels.setPixel(1, Colour(0, bumperBL * 255, 0), true);
    pixels.setPixel(2, Colour(0, bumperBL * 255, 0), true);
    pixels.setPixel(3, Colour(leftIRBrightness, bumperL * 255, 0), true);
    pixels.setPixel(4, Colour(0, bumperL * 255, 0), true);
    pixels.setPixel(5, Colour(0, bumperFL * 255, 0), true);
    pixels.setPixel(6, Colour(frontLeftIRBrightness, bumperFL * 255, 0), true);
    pixels.setPixel(7, Colour(0, bumperF * 255, ultrasonicBrightness), true);
    pixels.setPixel(8, Colour(0, bumperF * 255, ultrasonicBrightness), true);
    pixels.setPixel(9, Colour(frontRightIRBrightness, bumperFR * 255, 0), true);
    pixels.setPixel(10, Colour(0, bumperFR * 255, 0), true);
    pixels.setPixel(11, Colour(rightIRBrightness, bumperR * 255, 0), true);
    pixels.setPixel(12, Colour(0, bumperR * 255, 0), true);
    pixels.setPixel(13, Colour(0, bumperBR * 255, 0), true);
    pixels.setPixel(14, Colour(0, bumperBR * 255, 0), true);
    pixels.setPixel(15, Colour(0, bumperB * 255, 0), true);

    // Print the encoder values.
    Serial.print(" ENC1A:");
    Serial.print(digitalRead(LEFT_MOTOR_ENCODER_A_PIN));
    Serial.print(" ENC1B:");
    Serial.print(digitalRead(LEFT_MOTOR_ENCODER_B_PIN));
    Serial.print(" ENC1 Steps:");
    Serial.print(leftMotorSteps);

    Serial.print(" ENC2A:");
    Serial.print(digitalRead(RIGHT_MOTOR_ENCODER_A_PIN));
    Serial.print(" ENC2B:");
    Serial.print(digitalRead(RIGHT_MOTOR_ENCODER_B_PIN));
    Serial.print(" ENC2 Steps:");
    Serial.println(rightMotorSteps);

    NESInput nesReading = nesController.getNESInput();

    // If any NES buttons are pressed, print the state of the NES controller.
    if (nesReading.anyButtonPressed()) {
        Serial.println(nesReading);
    }

    bool upLeft = nesReading.buttonUp && nesReading.buttonLeft;
    bool upRight = nesReading.buttonUp && nesReading.buttonRight;
    bool downLeft = nesReading.buttonDown && nesReading.buttonLeft;
    bool downRight = nesReading.buttonDown && nesReading.buttonRight;

    if (upLeft) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 100);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 150);
    } else if (upRight) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 100);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 100);
    } else if (downLeft) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 100);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 150);
    } else if (downRight) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 150);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 100);
    } else if (nesReading.buttonUp) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 150);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 150);
    } else if (nesReading.buttonDown) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 150);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 150);
    } else if (nesReading.buttonLeft) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 100);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 100);
    } else if (nesReading.buttonRight) {
        digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
        digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
        analogWrite(LEFT_MOTOR_SPEED_PIN, 100);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 100);
    } else {  // If no NES direction buttons are pressed, stop the motors.
        analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
    }
    Serial.println();

    delay(100);
}
