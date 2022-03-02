#include <Adafruit_MPL3115A2.h>
#include <LoRa.h>


Adafruit_MPL3115A2 altimeter;

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  altimeter.setSeaPressure(1013.26);
}

void loop()
{
  float altitude = altimeter.getAltitude();
  Serial.print("altitude = "); Serial.print(altitude); Serial.println(" ft");
  delay(250);
}
