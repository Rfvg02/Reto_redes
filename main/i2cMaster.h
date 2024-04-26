#include <Wire.h>

#define SLAVE_ADDRESS 0x80
#define LAT_ADDRESS 0X01
#define LON_ADDRESS 0X02
#define ALT_ADDRESS 0X03
#define OBJ_LAT_ADDRESS 0X04

#define I2C_SDA 26
#define I2C_SCL 27

float i2cRead(uint8_t address) {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(address); // Escribe al ESP esclavo
  Wire.endTransmission(false);
  Wire.requestFrom(SLAVE_ADDRESS, 4, true); // La latitud es una variable flotante, por lo cual se solicitan 4 bytes

  union {
    float floatValue;
    uint8_t bytes[4];
  } data;

  for (int i = 0; i < 4; i++) {
    data.bytes[i] = Wire.read();
  }

  return data.floatValue;
}
