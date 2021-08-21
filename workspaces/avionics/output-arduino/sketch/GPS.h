#line 1 "/home/asimpoptani/work/yar-software/workspaces/avionics/esp32avonics/GPS.h"
struct GPS_struct
{
    double latitude;
    double longitude;
    double altitude;
    double speed;
    double course;
    int satellites;
    unsigned long lastUpdated;
};
struct Altimeter_struct {
    float altitude;
    float pressure;
    float temperature;
    unsigned long lastUpdated;
};

struct Accelrometer_struct {
    float x_accel;
    float y_accel;
    float z_accel;
    float x_gyro;
    float y_gyro;
    float z_gyro;
    float temperature;
    unsigned long lastUpdated;
};
