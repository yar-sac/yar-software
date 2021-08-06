#include <SD.h>
#include <SPI.h>

const int SPI_SD_SS = 3;

class DataLogger
{
private:
    File file;
    int maxSaves;
    int numberOfSaves;

public:
    DataLogger();
    ~DataLogger();
    void init(const char *name, int maxSaves);
    float alt;
    float pressure;
    float lat;
    float lon;
    float height;
    void tick();
};
