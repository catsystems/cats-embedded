# ArduinoLSM6DS3 library


The ArduinoLSM6DS3 library allows you to use the inertial measurement unit (IMU) available on the Arduino&reg; Nano 33 IoT board and the Arduino&reg; UNO WiFi Rev 2 board. The IMU is a [LSM6DS3](https://www.st.com/resource/en/datasheet/lsm6ds3.pdf), it has a 3-axis accelerometer and a 3-axis gyroscope. The IMU is connected through I2C on the Nano 33 IoT board's microcontroller and through SPI on the UNO WiFi Rev. 2 board's microcontroller. The values returned are signed floats.

To use this library:

```
#include <Arduino_LSM6DS3.h>
```

The ArduinoLSM6DS3 library takes care of the sensor initialization and sets its values as follows:

- Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
- Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
- Output data rate is fixed at 104 Hz.




