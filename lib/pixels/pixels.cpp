
#include "pixels.h"

#include <regex>

bool isValidHexColour(String stringToValidate) {
    const std::regex colourPattern(R"(^#([0-9A-Fa-f]{6})$)");
    return std::regex_match(stringToValidate.c_str(), colourPattern);
}

Colour::Colour(byte red, byte green, byte blue)
    : red(red), green(green), blue(blue) {}

void Colour::set(byte red, byte green, byte blue) {
    this->red = red;
    this->green = green;
    this->blue = blue;
}

String Colour::toString() {
    String stringToReturn = "";

    stringToReturn += "RBG(";
    stringToReturn += this->red;
    stringToReturn += ",";
    stringToReturn += this->green;
    stringToReturn += ",";
    stringToReturn += this->blue;
    stringToReturn += ")";

    return stringToReturn;
}

Pixels::Pixels(uint8_t dataPin, uint8_t ledCount)
    : _ledCount(ledCount), _ledStrip(_ledCount, dataPin) {}

void Pixels::setup() {
    this->_ledStrip.Begin();
    this->_ledStrip.Show();
}

void Pixels::setPixel(int index, Colour colour, bool show) {
    byte r = colour.red;
    byte g = colour.green;
    byte b = colour.blue;

    this->_ledStrip.SetPixelColor(index, RgbColor(r, g, b));

    if (show) {
        this->show();
    }
}

void Pixels::setAll(Colour colour, bool show) {
    for (uint8_t i = 0; i < this->_ledCount; i++) {
        this->setPixel(i, colour, show);
    }
}

void Pixels::clear(bool show) { this->setAll(Colour(0, 0, 0), show); }

void Pixels::show() { this->_ledStrip.Show(); }

uint16_t Pixels::getLedCount() { return this->_ledCount; }
