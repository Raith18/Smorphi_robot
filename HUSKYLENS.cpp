#include "HUSKYLENS.h"

HUSKYLENS::HUSKYLENS() : _available(false) {
}

bool HUSKYLENS::begin(Stream& serial) {
    _serial = &serial;
    // Initialize HuskyLens communication
    _serial->write(0x55); // Protocol header
    delay(100);
    return true;
}

bool HUSKYLENS::request() {
    if (!_serial) return false;

    // Send request command to HuskyLens
    _serial->write(0x2C); // Request frame command
    _available = false;
    return true;
}

bool HUSKYLENS::available() {
    // Check if data is available from HuskyLens
    if (_serial && _serial->available() >= 8) {
        // Read and parse HuskyLens data format
        // This is a simplified implementation
        uint8_t header = _serial->read();
        if (header == 0x2C) {
            uint8_t data[7];
            for (int i = 0; i < 7; i++) {
                data[i] = _serial->read();
            }

            // Parse the received data (simplified parsing)
            _result.xCenter = (data[0] << 8) | data[1];
            _result.yCenter = (data[2] << 8) | data[3];
            _result.width = (data[4] << 8) | data[5];
            _result.height = (data[6] << 8) | data[7];
            _result.ID = data[6]; // Use height as ID for demo

            _available = true;
            return true;
        }
    }
    return false;
}

HUSKYLENS::HUSKYLENSResult HUSKYLENS::read() {
    _available = false;
    return _result;
}