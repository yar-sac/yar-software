// Need to include Serial??
#include <Wire.h>
#include <SPI.h>
const int I2C_PRESSURE_AND_ALTIMETER=4
const int SPI_RADIO_SS=7
const int SPI_SD_SS=8
void setup() {
    // Setup Serial to talk to the GPS
    // https://www.u-blox.com/sites/default/files/products/documents/MAX-6_DataSheet_%28GPS.G6-HW-10106%29.pdf
    Serial1.begin(115200);
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
    char * gpsData=Serial1.readString()
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
}
    // Save to SD card
    digitalWrite(SPI_SD_SS, LOW); // enable Slave Select
    digitalWrite(SPI_SD_SS, HIGH`); // disable Slave Select
}
