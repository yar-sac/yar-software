#include "DataLogger.h"

DataLogger::DataLogger()
{
    
}

DataLogger::~DataLogger()
{
    file.close();
}

void DataLogger::init(const char *name, int maxSaves)
{
    Serial.print("DataLogger::initalising card...");

    if (!SD.begin(SPI_SD_SS))
    {
        Serial.println("DataLogger::initialisation failed!");
        while (1)
            ;
    }
    DataLogger::maxSaves = maxSaves;
    DataLogger::numberOfSaves = 0;

    DataLogger::file = SD.open(name, FILE_WRITE);

    Serial.println("DataLogger::initialisation done.");
}


void DataLogger::tick()
{
    char buffer[500];
    sprintf(buffer, "%.1f,%.1f,%.6f,%.6f,%.1f\n", alt, pressure, lat, lon, height);
    DataLogger::file.write(buffer);
    DataLogger::numberOfSaves += 1;
    if (DataLogger::numberOfSaves > DataLogger::maxSaves)
    {
        DataLogger::file.flush();
    }
}