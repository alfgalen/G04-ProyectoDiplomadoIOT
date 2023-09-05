#include <I2CSoilMoistureSensor.h>
#include <Wire.h>
void setup() {
  pinMode(GPIO1, OUTPUT);
  digitalWrite(GPIO1,HIGH);
  delay(1000);
  Wire.begin();
  Serial.begin(115200);

}

void loop() {
  byte error, address;
  int nDevices;
  while (Serial.available() != 0) {
    nDevices = 0;
    Serial.println("Ingresar nueva direccion");
    int menuChoice = Serial.parseInt();
    if (menuChoice != 0){
      Serial.print("Nueva direccion: ");
      Serial.println(menuChoice);
      Serial.println("Escaneando...");
      for(address = 10; address < 15; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    delay(1000);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Encontrado 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      
      nDevices++;
      I2CSoilMoistureSensor sensor(address);
      sensor.begin(); // reset sensor
      delay(1000); // give some time to boot up
      Serial.print("I2C Soil Moisture Sensor Address: ");
      Serial.println(sensor.getAddress(),HEX);
      Serial.print("Sensor Firmware version: ");
      Serial.println(sensor.getVersion(),HEX);
      Serial.println();

      Serial.print("Change address to... ");
      Serial.println(menuChoice);
      if (sensor.setAddress(menuChoice,true)){ // set Sensor Address to 0x21 and reset
        Serial.println("... DONE");
      }else{
        Serial.println("... ERROR");
      }
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
     }
  }
  delay(1000);
}
