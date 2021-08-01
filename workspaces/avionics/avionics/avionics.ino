// Need to include Serial??
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h>
TinyGPS gps;
const int I2C_PRESSURE_AND_ALTIMETER=4
const int SPI_RADIO_SS=7
const int SPI_SD_SS=8
void setup() {
    // Setup Serial to talk to the GPS
    // https://www.u-blox.com/sites/default/files/products/documents/MAX-6_DataSheet_%28GPS.G6-HW-10106%29.pdf
    Serial1.begin(9600);
    while (!Serial1);
    // Setup I2C to talk to the MPL3115A2 - Pressure sensor with altimetry
     Wire.begin(); // join i2c bus (address optional for master)
    // Setup SPI so we can talk to the radio and setup SD card SPI
    digitalWrite(SPI_SD_SS, HIGH); // disable Slave Select
    digitalWrite(SPI_RADIO_SS, HIGH); // disable Slave Select
    SPI.begin ();
    SPI.setClockDivider(SPI_CLOCK_DIV8);//divide the clock by 8
    
    
}

void loop() {
    // Get GPS data
    // https://github.com/neosarchizo/TinyGPS/blob/master/examples/simple_test/simple_test.ino
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
        while (Serial1.available())
        {
        char c = Serial1.read();
        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) // Did a new valid sentence come in?
            newData = true;
        }
    }

    if (newData)
    {
        float flat, flon;
        unsigned long age;
        gps.f_get_position(&flat, &flon, &age);
        Serial.print("LAT=");
        Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
        Serial.print(" LON=");
        Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
        Serial.print(" SAT=");
        Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
        Serial.print(" PREC=");
        Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    }
    
    gps.stats(&chars, &sentences, &failed);
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
    if (chars == 0){
        Serial.println("** No characters received from GPS: check wiring **");
    }
    // Get pressure and altitude
    Wire.beginTransmission(I2C_PRESSURE_AND_ALTIMETER); // transmit to device #4
    Wire.write("x is ");        // sends five bytes 
    Wire.endTransmission(); 
    int index =0;
    char pressureAndAltitude[50]={}
    // Read pressure and altitude 
    while(Wire.available())    // slave may send less than requested
    {
        char c = Wire.read();    // receive a byte as character
        pressureAndAltitude[index]=c; 
        index=index+1
    }
    // Combnine data to a string or CSV
    // Send via radio interface
    digitalWrite(SPI_RADIO_SS, LOW); // enable Slave Select
    digitalWrite(SPI_RADIO_SS, HIGH`); // disable Slave Select

    // Save to SD card
    digitalWrite(SPI_SD_SS, LOW); // enable Slave Select
    digitalWrite(SPI_SD_SS, HIGH`); // disable Slave Select
}
