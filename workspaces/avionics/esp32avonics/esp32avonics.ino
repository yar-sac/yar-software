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
TaskHandle_t GPSTask;
TaskHandle_t AltimeterTask;
TaskHandle_t AccelrometerTask;
// TaskHandle_t AccelrometerTask2;
TaskHandle_t RadioTask;
TaskHandle_t SDCardWriteTask;
TaskHandle_t SDCardWriteTask2;

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

// PORTS
const int SDCARD_SS_SPI=8;


void Accelrometer(void *pvParameters)
{

    Adafruit_MPU6050 mpu;
    Adafruit_MPU6050 mpu2;
    mpu.begin();
    mpu2.begin(0x69);
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu2.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu2.setFilterBandwidth(MPU6050_BAND_260_HZ);
    mpu2.setGyroRange(MPU6050_RANGE_2000_DEG);


    while (1)
    {
        sensors_event_t a, g, temp;
        xSemaphoreTake(i2c_mutex,portMAX_DELAY);
        mpu.getEvent(&a, &g, &temp);
        xSemaphoreGive(i2c_mutex);

        xSemaphoreTake(accelrometer_mutex,portMAX_DELAY);
        accelrometer_struct.x_accel=a.acceleration.x;
        accelrometer_struct.y_accel=a.acceleration.y;
        accelrometer_struct.z_accel=a.acceleration.z;
        accelrometer_struct.x_gyro=g.gyro.x;
        accelrometer_struct.y_gyro=g.gyro.y;
        accelrometer_struct.z_gyro=g.gyro.z;
        accelrometer_struct.temperature=temp.temperature;
        accelrometer_struct.lastUpdated=millis();
        xSemaphoreGive(accelrometer_mutex);
        // printf("%.3f ",temp.temperature);

        xSemaphoreTake(i2c_mutex,portMAX_DELAY);
        mpu2.getEvent(&a, &g, &temp);
        xSemaphoreGive(i2c_mutex);

        xSemaphoreTake(accelrometer_mutex2,portMAX_DELAY);
        accelrometer_struct2.x_accel=a.acceleration.x;
        accelrometer_struct2.y_accel=a.acceleration.y;
        accelrometer_struct2.z_accel=a.acceleration.z;
        accelrometer_struct2.x_gyro=g.gyro.x;
        accelrometer_struct2.y_gyro=g.gyro.y;
        accelrometer_struct2.z_gyro=g.gyro.z;
        accelrometer_struct2.temperature=temp.temperature;
        accelrometer_struct2.lastUpdated=millis();
        xSemaphoreGive(accelrometer_mutex2);
        // printf("%.3f\n",temp.temperature);
        delay(10);
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
        while (Serial2.available()){
            // printf("READING %c",Serial2.read());
            tinyGPS.encode(Serial2.read());
        }
        // printf("heyo");

        xSemaphoreTake(gps_mutex,portMAX_DELAY);
        gps_struct.altitude=tinyGPS.altitude.meters();
        gps_struct.latitude=tinyGPS.location.lat();
        gps_struct.longitude=tinyGPS.location.lng();
        gps_struct.lastUpdated=millis();
        gps_struct.speed=tinyGPS.speed.kmph();
        gps_struct.course=tinyGPS.course.deg();
        gps_struct.satellites=tinyGPS.satellites.value();

        xSemaphoreGive(gps_mutex);
        delay(100);
    }
}
void Altimeter(void *pvParameters)
{
    Adafruit_BMP280 altimeter= Adafruit_BMP280(Wire.begin(33,32));
     if (!altimeter.begin(BMP280_ADDRESS_ALT,BMP280_CHIPID)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  altimeter.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    
    while (1)
    {
        xSemaphoreTake(altimeter_mutex,portMAX_DELAY);
        altimeter_stuct.altitude=altimeter.readAltitude();;
        altimeter_stuct.lastUpdated=millis();
        xSemaphoreGive(altimeter_mutex);
        delay(100);
        printf("%.2f %.2f\n",altimeter.readAltitude(),altimeter.readTemperature());
    }
}

void generateString(char *buffer){
  // Altimeter
        Altimeter_struct altimeter_stuct_copy;
        xSemaphoreTake(altimeter_mutex,portMAX_DELAY);
        memcpy(&altimeter_stuct_copy,&altimeter_stuct,sizeof(Altimeter_struct));
        xSemaphoreGive(altimeter_mutex);
        // GPS
        GPS_struct gps_struct_copy;
        xSemaphoreTake(gps_mutex,portMAX_DELAY);
        memcpy(&gps_struct_copy,&gps_struct,sizeof(GPS_struct));
        xSemaphoreGive(gps_mutex);
        // Accelrometers
        Accelrometer_struct accelrometer_struct_copy;
        Accelrometer_struct accelrometer_struct2_copy;
        xSemaphoreTake(accelrometer_mutex,portMAX_DELAY);
        memcpy(&accelrometer_struct_copy,&accelrometer_struct,sizeof(Accelrometer_struct));
        xSemaphoreGive(accelrometer_mutex);
        xSemaphoreTake(accelrometer_mutex2,portMAX_DELAY);
        memcpy(&accelrometer_struct2_copy,&accelrometer_struct2,sizeof(Accelrometer_struct));
        xSemaphoreGive(accelrometer_mutex2);

        sprintf(buffer,"%.5f, %lu, %.5f, %.5f, %.5f, %d, %.5f, %lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f , %lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %lu \n",
        altimeter_stuct_copy.altitude
        ,altimeter_stuct_copy.lastUpdated
        ,gps_struct_copy.altitude
        ,gps_struct_copy.course
        ,gps_struct_copy.latitude
        ,gps_struct_copy.longitude
        ,gps_struct_copy.satellites
        ,gps_struct_copy.speed
        ,gps_struct_copy.lastUpdated
        ,accelrometer_struct_copy.x_gyro
        ,accelrometer_struct_copy.y_gyro
        ,accelrometer_struct_copy.z_gyro
        ,accelrometer_struct_copy.x_accel
        ,accelrometer_struct_copy.y_accel
        ,accelrometer_struct_copy.z_accel
        ,accelrometer_struct_copy.temperature
        ,accelrometer_struct_copy.lastUpdated
        ,accelrometer_struct2_copy.x_gyro
        ,accelrometer_struct2_copy.y_gyro
        ,accelrometer_struct2_copy.z_gyro
        ,accelrometer_struct2_copy.x_accel
        ,accelrometer_struct2_copy.y_accel
        ,accelrometer_struct2_copy.z_accel
        ,accelrometer_struct2_copy.temperature
        ,accelrometer_struct2_copy.lastUpdated
        );

}

void Radio(void *pvParameters)
{
    while (1)
    {
        char buffer[1000];
        generateString(buffer);
        printf(buffer);
      
        // printf("Lat %.5f Lng %.5f Alt %.5f Sat %d\n", copy2.latitude,copy2.longitude,copy2.altitude,copy2.satellites);

        delay(10);

    }
}
void SDCardWrite(void *pvParameters)
{
    SDFS SDCard = SDFS(FSImplPtr(new VFSImpl()));
    while  (!SDCard.begin(SDCARD_SS_SPI)){
        Serial.println("SD Card not detected");
        delay(200);
    }
    Serial.println("SD CARD detected");
    while (SDCard.cardType()==CARD_NONE){
        Serial.println("No SD card detected");
        delay(200);
    }
    switch (SDCard.cardType()){
        case CARD_MMC:
        Serial.println("MMC Card detected");
        break;
        case CARD_SD:
        Serial.println("SD CARD detected");
        break;
        case CARD_SDHC:
        printf("SDHC card detected\n");
        break;
        case CARD_UNKNOWN:
        printf("Unkown card detected\n");
        break;
        default:
        printf("SOMETHING has gone seriously wrong");
        break;
    }
    testFileIO(SDCard,"test.txt");
    while (1)
    {
        char buffer[1000];
        generateString(buffer);
        appendFile(SDCard,"results.txt",buffer);
        delay(100);

        // printf("Running task SDCardWrite\n");
        // printf("Hello world from core %d!\n", xPortGetCoreID() );
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void SDCardWrite2(void *pvParameters)
{
    while (1)
    {
        // printf("Running task SDCardWrite\n");
        // printf("Hello world from core %d!\n", xPortGetCoreID() );
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Wire.begin();
    Wire.setClock(400000);
    // Serial begin
    Serial.begin(115200);
    while (!Serial)
        ;
    //   Create mutexs
    gps_mutex = xSemaphoreCreateMutex();
    accelrometer_mutex = xSemaphoreCreateMutex();
    altimeter_mutex = xSemaphoreCreateMutex();
    accelrometer_mutex2 = xSemaphoreCreateMutex();
    i2c_mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(GPS, "GPS", 10000, NULL, 1, &GPSTask, 1);
    xTaskCreatePinnedToCore(Altimeter, "Altimeter", 10000, NULL, 1, &AltimeterTask, 1);
    xTaskCreatePinnedToCore(Radio, "Radio", 10000, NULL, 1, &RadioTask, 0);
    xTaskCreatePinnedToCore(SDCardWrite, "SDCard1", 10000, NULL, 1, &SDCardWriteTask, 1);
    xTaskCreatePinnedToCore(SDCardWrite2, "SDCard2", 10000, NULL, 1, &SDCardWriteTask2, 1);
    xTaskCreatePinnedToCore(Accelrometer, "Accel1", 10000, NULL, 1, &AccelrometerTask, 1);
}

void loop()
{
}
