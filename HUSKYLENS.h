#ifndef HUSKYLENS_H
#define HUSKYLENS_H

#include <Arduino.h>

class HUSKYLENS {
public:
    struct HUSKYLENSResult {
        int xCenter;
        int yCenter;
        int width;
        int height;
        int ID;
    };

    HUSKYLENS();
    bool begin(Stream& serial);
    bool request();
    bool available();
    HUSKYLENSResult read();

private:
    Stream* _serial;
    HUSKYLENSResult _result;
    bool _available;
};

#endif