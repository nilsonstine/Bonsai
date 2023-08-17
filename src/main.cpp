#include <Arduino.h>
#include <drv2605.h>
#include "MAX30105.h" //Get it here: http://librarymanager/All#SparkFun_MAX30105
#include "heartRate.h"
#include <AverageValue.h>

// Number of values to calculate with. Prevents memory problems
const long MAX_VALUES_NUM = 5;

AverageValue<long> averageValue(MAX_VALUES_NUM);

MAX30105 particleSensor;

DRV2605 haptic;

#include <SimpleKalmanFilter.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Heartbeat Vars
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

void setupHaptic()
{
    Serial.println("DRV2605L Haptic Motor Test");

    if (haptic.init(false, true) != 0)
    {
        Serial.println("init failed!");
    }

    if (haptic.drv2605_AutoCal() != 0)
    {
        Serial.println("auto calibration failed!");
    }
}
void setupHeart()
{
    Serial.println("MAX30105 Blood Oxygen Sensor Test");
    // Initialize sensor
    if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) // Use default I2C port, 400kHz speed
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1)
            ;
    }

    // The LEDs are very low power and won't affect the temp reading much but
    // you may want to turn off the LEDs to avoid any local heating
    particleSensor.setup(); // Configure sensor. Turn off LEDs
    // particleSensor.setup(); //Configure sensor. Use 25mA for LED drive

    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
}
void setup()
{
    Serial.begin(115200);
    setupHaptic();
    setupHeart();
    delay(500);
}

void loop()
{
    // static unsigned long lastTime = 0;
    // const unsigned long interval = 2000;
    // unsigned long currentTime = millis();
    // if (currentTime - lastTime >= interval)
    //{
    //     lastTime = currentTime;
    //     haptic.drv2605_Play_Waveform(118);
    // }
    long irRaw = particleSensor.getIR();
    long irValue = simpleKalmanFilter.updateEstimate(irRaw);

    if (checkForBeat(irValue) == true)
    {

        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
            averageValue.push(beatsPerMinute);
        }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(averageValue.average());

    if (irValue < 50000)
        Serial.print(" No finger?");

    // Calculate haptic interval based on average BPM
    int avgBpm = averageValue.average();
    if (avgBpm >= 20 && avgBpm <= 255 && irValue > 50000)
    {                                        // Adjust the range as needed
        int hapticInterval = 60000 / avgBpm; // Calculate interval based on average BPM
        static unsigned long lastHapticTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastHapticTime >= hapticInterval)
        {
            lastHapticTime = currentTime;
            haptic.drv2605_Play_Waveform(17); // Play the heartbeat waveform
        }
    }
    Serial.println();
}
// unsigned char i;
// for(i=1;i<124;i++)
// {
//     Serial.print("Effect No: ");
//     Serial.println(i);
//
//     haptic.drv2605_Play_Waveform(i);
//     delay(2000);
// }
