Modified [BME68X Sensor API](https://github.com/boschsensortec/BME68x_SensorAPI) example for ESP32-C3. My setup only supports I2C, so I didn't provide adapter functions for SPI. 

This example uses high resolution ESP timer and POSIX threads API wrapper to achieve microsecond resolution delays.