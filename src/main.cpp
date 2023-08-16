#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>

Adafruit_DRV2605 drv;

void setup()
{
    Serial.begin(115200);

    if (!drv.begin())
    {
        Serial.println("Failed to find DRV2605, check wiring!");
        while (1)
            ;
    }

    drv.selectLibrary(1); // Choose the LRA library

    drv.setMode(DRV2605_MODE_REALTIME); // Set mode to real-time control

    Serial.println("Testing all haptic effects...");

    Serial.println("Testing complete.");
}

void loop()
{
    drv.setWaveform(0, 25); // play effect
    drv.setWaveform(1, 0);      // end waveform

    // play the effect!
    drv.go();

    // wait a bit
    delay(500);
}
