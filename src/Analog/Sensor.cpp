/**
 * @file Sensor.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog sensor implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Analog/Sensor.hpp"

#include <stdint.h>

#include <Arduino.h>

using namespace Analog;

/**
 * @brief Construct a new analog sensor object
 *
 * @param inputPin Pin to read raw analog values
 * @param enablePin Enable sensor pin (active HIGH)
 */
Sensor::Sensor(uint8_t inputPin, uint8_t enablePin)
    : _inputPin(inputPin),
      _enablePin(enablePin)
{
}

/**
 * @brief Initialize analog sensor
 */
void Sensor::initialize()
{
    // Set input pin as input
    pinMode(_inputPin, INPUT);

    // Set enable pin as output
    pinMode(_enablePin, OUTPUT);
}

/**
 * @brief Enable analog sensor
 */
void Sensor::enable()
{
    // Power up sensor
    digitalWrite(_enablePin, HIGH);
}

/**
 * @brief Disable analog sensor
 */
void Sensor::disable()
{
    // Power down sensor
    digitalWrite(_enablePin, LOW);
}

/**
 * @brief Read current value from analog sensor
 *
 * @return Analog value
 */
uint16_t Sensor::read()
{
    uint16_t value = analogRead(_inputPin);

    return value;
}
