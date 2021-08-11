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
