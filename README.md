# Reto de Redes Industriales
Este repositorio contiene los códigos de los componentes del reto de búsqueda del tesoro, tanto el dispositivo de búsqueda 'master', como los dispositivos a buscar 'slaves'.

A continuación se detallan las funciones del 'master' para extraer datos usando diferentes protocolos de comunicación
## i2c
### Generalidades
El esclavo de i2c está configurado con una dirección `0x80`. Además de direcciones de registros `0x01`,`0x02`,`0x03`, y `0x04`, que corresponden respectivamente a **latitud**, **longitud**, **altitud**, y **latitud del objetivo final**

El módulo que contiene las funciones del master es `i2cMaster.h`
### Funciones
#### i2cRead(address)
Esta función escribe al registro correspondiente a una coordenada y entrega el valor respectivo.
- **Entradas**:
  - *address (uint8_t)*: La dirección del registro que dispara la lectura de la coordenada
- **Salidas**
  - *data (float)*: Valor del la coordenada
### Ejemplo de uso
```ino
#include <Wire.h>
#include <i2cMaster.h>
#define LAT_ADDRESS 0X01 //Ya están definidas en i2cMaster.h, pero es mejor dejarlas por claridad
#define LON_ADDRESS 0X02
#define ALT_ADDRESS 0X03
#define OBJ_LAT_ADDRESS 0X04

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  Serial.print("Latitud = ");
  latitud = i2cRead(LAT_ADDRESS);
  Serial.println(latitud, 4); //4 dígitos después del punto es suficiente presición para la tarea. Ver https://xkcd.com/2170/

  Serial.print("Longitud = ");
  longitud = i2cRead(LON_ADDRESS);
  Serial.println(longitud, 4);

  Serial.print("Altitud = ");
  altitud = i2cRead(ALT_ADDRESS);
  Serial.println(altitud, 4);

  Serial.print("Latitud Objetivo = ");
  latitudObjetivo = i2cRead(OBJ_LAT_ADDRESS);
  Serial.println(latitudObjetivo, 4);

  delay(1000);
}
```
## Bluetooth
### Generalidades
### Funciones
### Ejemplo de uso

## TCP/IP
### Generalidades
### Funciones
### Ejemplo de uso
