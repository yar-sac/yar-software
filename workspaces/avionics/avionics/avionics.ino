// Need to include Serial??

void setup() {
    // Setup Serial to talk to the GPS
    // https://www.u-blox.com/sites/default/files/products/documents/MAX-6_DataSheet_%28GPS.G6-HW-10106%29.pdf
    Serial1.begin(115200);
    while (!Serial1);
    // Setup I2C to talk to the MPL3115A2 - Pressure sensor with altimetry
    // Setup SPI so we can talk to the radio
    // Setup SD card ????

}

void loop() {
    // Get GPS data
    // Get pressure and altitude
    // Combnine data to a string or CSV
    // Send via radio interface
    // Save to SD card
}
