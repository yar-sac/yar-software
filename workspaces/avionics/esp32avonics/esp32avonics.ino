/**
 * 
 **/
#define DEBUG_SERIAL Serial
#include "GPS.h"
#include <Wire.h> //include Wire.h library
#include <Adafruit_MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> // Include the TinyGPS++ library
#include <Adafruit_BMP280.h>
#include "SD.h"
#include "IOSdcard.h"
//#include <RF24.h>
#include <esp_now.h>
#include "WiFi.h"
TaskHandle_t GPSTask;
TaskHandle_t AltimeterTask;
TaskHandle_t AccelrometerTask;
// TaskHandle_t AccelrometerTask2;
TaskHandle_t RadioTask;
TaskHandle_t SDCardWriteTask;
TaskHandle_t SDCardWriteTask2;
TaskHandle_t BlinkTask;

GPS_struct gps_struct = {};
Altimeter_struct altimeter_stuct = {};
Accelrometer_struct accelrometer_struct = {};
Accelrometer_struct accelrometer_struct2 = {};

// For structs
SemaphoreHandle_t gps_mutex;
SemaphoreHandle_t altimeter_mutex;
SemaphoreHandle_t accelrometer_mutex;
SemaphoreHandle_t accelrometer_mutex2;

// For I2C
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t accel_i2c_mutex;
SemaphoreHandle_t spi_mutex;

    TwoWire I2Cone = TwoWire(0);
    TwoWire I2Ctwo = TwoWire(1);

// PORTS
const int SDCARD_SS_SPI = 33;
const int SDCARD2_SS_SPI = 34;

const int LED_PIN = 2;

void Accelrometer(void *pvParameters)
{

    Adafruit_MPU6050 mpu;
    // Adafruit_MPU6050 mpu2;
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    mpu.begin(0x68,&I2Cone);
    // mpu2.begin(0x69,&I2Cone);
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    // mpu2.setAccelerometerRange(MPU6050_RANGE_16_G);
    // mpu2.setFilterBandwidth(MPU6050_BAND_260_HZ);
    // mpu2.setGyroRange(MPU6050_RANGE_2000_DEG);
    xSemaphoreGive(i2c_mutex);

    while (1)
    {
        sensors_event_t a, g, temp;
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        mpu.getEvent(&a, &g, &temp);
        xSemaphoreGive(i2c_mutex);

        xSemaphoreTake(accelrometer_mutex, portMAX_DELAY);
        accelrometer_struct.x_accel = a.acceleration.x;
        accelrometer_struct.y_accel = a.acceleration.y;
        accelrometer_struct.z_accel = a.acceleration.z;
        accelrometer_struct.x_gyro = g.gyro.x;
        accelrometer_struct.y_gyro = g.gyro.y;
        accelrometer_struct.z_gyro = g.gyro.z;
        accelrometer_struct.temperature = temp.temperature;
        accelrometer_struct.lastUpdated = millis();
        xSemaphoreGive(accelrometer_mutex);
        
        
        // xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        // sensors_event_t a2, g2, temp2;
        // mpu2.getEvent(&a2, &g2, &temp2);
        // xSemaphoreGive(i2c_mutex);

        // xSemaphoreTake(accelrometer_mutex2, portMAX_DELAY);
        // accelrometer_struct2.x_accel = a2.acceleration.x;
        // accelrometer_struct2.y_accel = a2.acceleration.y;
        // accelrometer_struct2.z_accel = a2.acceleration.z;
        // accelrometer_struct2.x_gyro = g2.gyro.x;
        // accelrometer_struct2.y_gyro = g2.gyro.y;
        // accelrometer_struct2.z_gyro = g2.gyro.z;
        // accelrometer_struct2.temperature = temp2.temperature;
        // accelrometer_struct2.lastUpdated = millis();
        // xSemaphoreGive(accelrometer_mutex2);
        
        // printf("%.3f\n",temp.temperature);
        // delay(5);
    }
}

void GPS(void *pvParameters)
{
    // SoftwareSerial gps_serial;
    // gps_serial.begin(9600,SWSERIAL_8N1,25,26);
    Serial2.begin(9600);
    TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

    while (1)
    {
        while (Serial2.available())
        {
            // printf("READING %c",Serial2.read());
            tinyGPS.encode(Serial2.read());
        }

        xSemaphoreTake(gps_mutex, portMAX_DELAY);
        gps_struct.altitude = tinyGPS.altitude.meters();
        gps_struct.latitude = tinyGPS.location.lat();
        gps_struct.longitude = tinyGPS.location.lng();
        gps_struct.lastUpdated = millis();
        gps_struct.speed = tinyGPS.speed.kmph();
        gps_struct.course = tinyGPS.course.deg();
        gps_struct.satellites = tinyGPS.satellites.value();

        gps_struct.hour = tinyGPS.time.hour();
        gps_struct.min = tinyGPS.time.minute();
        gps_struct.sec = tinyGPS.time.second();
        gps_struct.centi = tinyGPS.time.centisecond();

        xSemaphoreGive(gps_mutex);
        delay(100);
    }
}
void Altimeter(void *pvParameters)
{

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    Adafruit_BMP280 bmp = Adafruit_BMP280(&I2Cone); // use I2C interface
    Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
    Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
    xSemaphoreGive(i2c_mutex);

    //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    if (!bmp.begin())
    {
        // Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
        //                  "try a different address!"));
        while (1)
            delay(10);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    while (1)
    {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        sensors_event_t temp_event, pressure_event;
        bmp_temp->getEvent(&temp_event);
        bmp_pressure->getEvent(&pressure_event);
        xSemaphoreGive(altimeter_mutex);
        altimeter_stuct.pressure = pressure_event.pressure;
        altimeter_stuct.altitude = bmp.readAltitude(1020.0);
        altimeter_stuct.temperature = temp_event.temperature;
        altimeter_stuct.lastUpdated = millis();
        xSemaphoreGive(altimeter_mutex);
        xSemaphoreGive(i2c_mutex);
        delay(8);
    }
}

void generateString(char *buffer)
{
    // Altimeter
    Altimeter_struct altimeter_stuct_copy;
    xSemaphoreTake(altimeter_mutex, portMAX_DELAY);
    memcpy(&altimeter_stuct_copy, &altimeter_stuct, sizeof(Altimeter_struct));
    xSemaphoreGive(altimeter_mutex);
    // GPS
    GPS_struct gps_struct_copy;
    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    memcpy(&gps_struct_copy, &gps_struct, sizeof(GPS_struct));
    xSemaphoreGive(gps_mutex);
    // Accelrometers
    Accelrometer_struct accelrometer_struct_copy;
    Accelrometer_struct accelrometer_struct2_copy;
    xSemaphoreTake(accelrometer_mutex, portMAX_DELAY);
    memcpy(&accelrometer_struct_copy, &accelrometer_struct, sizeof(Accelrometer_struct));
    xSemaphoreGive(accelrometer_mutex);
    xSemaphoreTake(accelrometer_mutex2, portMAX_DELAY);
    memcpy(&accelrometer_struct2_copy, &accelrometer_struct2, sizeof(Accelrometer_struct));
    xSemaphoreGive(accelrometer_mutex2);
    // printf("%lf %lf",gps_struct_copy.latitude,gps_struct_copy.longitude);
    // sprintf(buffer, "%f, %f, %f ,%lu, %lf, %lf, %lf, %lf, %d, %lf, %lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f , %lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %lu \n",
    sprintf(buffer, "%d, %d, %d, %d, %f, %f, %f ,%lu, %lf, %lf, %lf, %lf, %d, %lf, %lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f , %lu \n",
            gps_struct_copy.hour,
            gps_struct_copy.min,
            gps_struct_copy.sec,
            gps_struct_copy.centi,
            altimeter_stuct_copy.altitude,
            altimeter_stuct_copy.pressure,
            altimeter_stuct_copy.temperature,
            altimeter_stuct_copy.lastUpdated,
            gps_struct_copy.altitude,
            gps_struct_copy.course,
            gps_struct_copy.latitude,
            gps_struct_copy.longitude,
            gps_struct_copy.satellites,
            gps_struct_copy.speed,
            gps_struct_copy.lastUpdated,
            accelrometer_struct_copy.x_gyro,
            accelrometer_struct_copy.y_gyro,
            accelrometer_struct_copy.z_gyro,
            accelrometer_struct_copy.x_accel,
            accelrometer_struct_copy.y_accel,
            accelrometer_struct_copy.z_accel,
            accelrometer_struct_copy.temperature,
            accelrometer_struct_copy.lastUpdated
//  accelrometer_struct2_copy.x_gyro,
//  accelrometer_struct2_copy.y_gyro,
//  accelrometer_struct2_copy.z_gyro,
//  accelrometer_struct2_copy.x_accel,
//  accelrometer_struct2_copy.y_accel,
//  accelrometer_struct2_copy.z_accel,
//  accelrometer_struct2_copy.temperature,
//  accelrometer_struct2_copy.lastUpdated
 );
}
// Structure example to receive data
// Must match the sender structure

// void Radio(void *pvParameters)
// {

//   WiFi.mode(WIFI_MODE_STA);
//     // Init ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     // Serial.println("Error initializing ESP-NOW");
//     return;
//   }
//     // callback when data is sent
//     void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     Serial.print("\r\nLast Packet Send Status:\t");
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//     }
  
//   // Once ESPNow is successfully Init, we will register for recv CB to
//   // get recv packer info
//   esp_now_register_recv_cb(OnDataRecv);
//     while (1)
//     {
        
//         char buffer[250];



//         delay(10);
//     }
// }
void SDCardWrite(void *pvParameters)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    SDFS SDCard = SDFS(FSImplPtr(new VFSImpl()));
    while (!SDCard.begin(SDCARD_SS_SPI))
    {
        // Serial.println("SD Card not detected");
        delay(200);
    }
    xSemaphoreGive(spi_mutex);
    // Serial.println("SD CARD detected");
    // while (SDCard.cardType()==CARD_NONE){
    //     Serial.println("No SD card detected");
    //     delay(200);
    // }
    // switch (SDCard.cardType()){
    //     case CARD_MMC:
    //     Serial.println("MMC Card detected");
    //     break;
    //     case CARD_SD:
    //     Serial.println("SD CARD detected");
    //     break;
    //     case CARD_SDHC:
    //     printf("SDHC card detected\n");
    //     break;
    //     case CARD_UNKNOWN:
    //     printf("Unkown card detected\n");
    //     break;
    //     default:
    //     printf("SOMETHING has gone seriously wrong");
    //     break;
    // }
    // testFileIO(SDCard,"/test.txt");
    // readFile(SDCard,"/results.txt");
    // delay(500);
    while (1)
    {
        char buffer[1000];
        generateString(buffer);
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        // printf("Writing to results");
        appendFile(SDCard, "/results.txt", buffer);
        xSemaphoreGive(spi_mutex);
        delay(1);

        // printf("Running task SDCardWrite\n");
        // printf("Hello world from core %d!\n", xPortGetCoreID() );
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void SDCardWrite2(void *pvParameters)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    SDFS SDCard2 = SDFS(FSImplPtr(new VFSImpl()));
    
    while (!SDCard2.begin(SDCARD2_SS_SPI))
    {
        Serial.println("SD Card 1 not detected");
        delay(200);
    }
    xSemaphoreGive(spi_mutex);
    while (1)
    {
        char buffer[1000];
        generateString(buffer);
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        printf("Writing to results 2");
        appendFile(SDCard2, "/results2.txt", buffer);
        xSemaphoreGive(spi_mutex);
        delay(1000);
        // printf("Running task SDCardWrite\n");
        // printf("Hello world from core %d!\n", xPortGetCoreID() );
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void StatusBlink(void *pvParameters)
{
    // Status blinker runs once a minute after boot
    // and gives a long or short blink for each sensor set
    // to show if that sensor is operating as expected.
    
    // Get all sensors' data
    // Altimeter
    Altimeter_struct altimeter_stuct_copy;
    xSemaphoreTake(altimeter_mutex, portMAX_DELAY);
    memcpy(&altimeter_stuct_copy, &altimeter_stuct, sizeof(Altimeter_struct));
    xSemaphoreGive(altimeter_mutex);
    // GPS
    GPS_struct gps_struct_copy;
    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    memcpy(&gps_struct_copy, &gps_struct, sizeof(GPS_struct));
    xSemaphoreGive(gps_mutex);
    // Accelrometers
    Accelrometer_struct accelrometer_struct_copy;
    xSemaphoreTake(accelrometer_mutex, portMAX_DELAY);
    memcpy(&accelrometer_struct_copy, &accelrometer_struct, sizeof(Accelrometer_struct));
    xSemaphoreGive(accelrometer_mutex);

    // Check that each sensor is operating as expected
    // Altimeter
    int alt = ((-100 < altimeter_stuct_copy.altitude) && (altimeter_stuct_copy.altitude < 1000));
    int pre = ((0 < altimeter_stuct_copy.pressure) && (altimeter_stuct_copy.pressure < 100000));
    int temp = ((10 < altimeter_stuct_copy.temperature) && (altimeter_stuct_copy.temperature < 35));
    int alti_working = alt && pre && temp;

    // GPS (Assuming we're still in the UK)
    int gp_alt = ((-100 < gps_struct_copy.altitude) && (gps_struct_copy.altitude < 1000));
    int lat = ((45 < gps_struct_copy.latitude) && (gps_struct_copy.latitude < 60));
    int lng = ((-10 < gps_struct_copy.longitude) && (gps_struct_copy.longitude < 10));
    int gps_working = gp_alt && lat && lng;

    // Accelrometers
    int x = ((-50 < accelrometer_struct_copy.x_accel) && (accelrometer_struct_copy.x_accel < 50));
    int y = ((-50 < accelrometer_struct_copy.y_accel) && (accelrometer_struct_copy.y_accel < 50));
    int z = ((-50 < accelrometer_struct_copy.z_accel) && (accelrometer_struct_copy.z_accel < 50));
    int acc_working = x && y && z;

    // Now we flash the light accordingly
    digitalWrite(LED_PIN,HIGH);
    delay(1000);
    digitalWrite(LED_PIN,LOW);
    delay(1000);
    digitalWrite(LED_PIN,HIGH);
    delay(alti_working ? 1000 : 200);    // Altimeter blink
    digitalWrite(LED_PIN,LOW);
    delay(1000);
    digitalWrite(LED_PIN,HIGH);
    delay(gps_working ? 1000 : 200);    // GPS blink
    digitalWrite(LED_PIN,LOW);
    delay(1000);
    digitalWrite(LED_PIN,HIGH);
    delay(acc_working ? 1000 : 200);    // Accel blink
    digitalWrite(LED_PIN,LOW);

    delay(20000); // Run again in 20
}

void setup()
{

    I2Cone.begin(21,22,400000);
    // I2Ctwo.begin(26, 25, 400000);
    // Serial begin
    Serial.begin(115200);
    // while (!Serial)
    //     ;
    //   Create mutexs
    gps_mutex = xSemaphoreCreateMutex();
    accelrometer_mutex = xSemaphoreCreateMutex();
    altimeter_mutex = xSemaphoreCreateMutex();
    accelrometer_mutex2 = xSemaphoreCreateMutex();
    i2c_mutex = xSemaphoreCreateMutex();
    accel_i2c_mutex = xSemaphoreCreateMutex();
    spi_mutex = xSemaphoreCreateMutex();


    pinMode(LED_PIN, OUTPUT);

    xTaskCreatePinnedToCore(GPS, "GPS", 10000, NULL, 1, &GPSTask, 0);
    xTaskCreatePinnedToCore(Altimeter, "Altimeter", 10000, NULL, 1, &AltimeterTask, 1);
    // xTaskCreatePinnedToCore(Radio, "Radio", 10000, NULL, 1, &RadioTask, 0);
    xTaskCreatePinnedToCore(SDCardWrite, "SDCard1", 10000, NULL, 1, &SDCardWriteTask, 1);
    // xTaskCreatePinnedToCore(SDCardWrite2, "SDCard2", 10000, NULL, 1, &SDCardWriteTask2, 1);
    xTaskCreatePinnedToCore(Accelrometer, "Accel1", 10000, NULL, 1, &AccelrometerTask, 1);
    xTaskCreatePinnedToCore(StatusBlink, "Blink", 10000, NULL, 1, &BlinkTask, 0);
}

void loop()
{
    // char buffer[1000];
    // generateString(buffer);
    // printf(buffer);
    // delay(1000);
      
}
