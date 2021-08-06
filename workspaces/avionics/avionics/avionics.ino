#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <SD.h>

#include "DataLogger.h"

#define SerialGPS Serial1

const int SAVE_COUNTER = 10;
const int I2C_PRESSURE_AND_ALTIMETER = 0xC0;
const int SPI_RADIO_SS = 7;

TinyGPS gps;
DataLogger dataLogger;

void setup()
{
    // Setup SerialUSB to talk to the GPS
    // https://www.u-blox.com/sites/default/files/products/documents/MAX-6_DataSheet_%28GPS.G6-HW-10106%29.pdf
    SerialGPS.begin(9600);

    SerialUSB.begin(9600);

    while (!SerialGPS)
        ;
    // Good for debugging, need to be removed later.
    while (!SerialUSB)
        ;

    SerialUSB.write("Started up\n");

    dataLogger.init("test.csv", 10);
}

void loop()
{
    // Get GPS data
    // https://github.com/neosarchizo/TinyGPS/blob/master/examples/simple_test/simple_test.ino
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 100;)
    {
        while (SerialGPS.available())
        {
            char c = SerialGPS.read();
            // SerialUSB.write(c); // uncomment this line if you want to see the GPS data flowing
            if (gps.encode(c)) // Did a new valid sentence come in?
                newData = true;
        }
    }

    if (newData)
    {
        // GPS lat and long
        float flat, flon;
        unsigned long age;
        gps.f_get_position(&flat, &flon, &age);
        SerialUSB.print("LAT=");
        SerialUSB.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);

        SerialUSB.print(" LON=");
        SerialUSB.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
        SerialUSB.print(" SAT=");
        SerialUSB.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
        SerialUSB.print(" PREC=");
        SerialUSB.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
        dataLogger.lat = flat;
        dataLogger.lon = flon;
    }

    gps.stats(&chars, &sentences, &failed);
    SerialUSB.print(" CHARS=");
    SerialUSB.print(chars);
    SerialUSB.print(" SENTENCES=");
    SerialUSB.print(sentences);
    SerialUSB.print(" CSUM ERR=");
    SerialUSB.println(failed);
    if (chars == 0)
    {
        SerialUSB.println("** No characters received from GPS: check wiring **");
    }

    // // Combnine data to a string or CSV
    // // Send via radio interface
    // digitalWrite(SPI_RADIO_SS, LOW); // enable Slave Select

    // // Talk to the radio
    // digitalWrite(SPI_RADIO_SS, HIGH); // disable Slave Select
    dataLogger.alt = 0;
    dataLogger.height = 0;

    dataLogger.pressure = 0;

    dataLogger.tick();
}
