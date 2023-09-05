#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Wire.h"
#include "I2CSoilMoistureSensor.h"
/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA, credenciales generadas en plataforma IOT*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x08, 0xF4 };
uint8_t appEui[] = { 0x55, 0x77, 0x00, 0x99, 0x44, 0x11, 0x00, 0x22 };
uint8_t appKey[] = { 0xE0, 0xB4, 0xD5, 0xF1, 0xD1, 0x73, 0xA7, 0xC1, 0x70, 0xF3, 0x5A, 0x26, 0xD7, 0x41, 0xBC, 0xEA };

/* ABP*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

//LoraWan channelsmask, default channels 0-7/ 
uint16_t userChannelsMask[6]={ 0x0000,0x00FF,0x0000,0x0000,0x0000,0x0000 };//FSB3, subBanda 3

//LoraWan region, select in arduino IDE tools/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

//LoraWan Class, Class A and Class C are supported/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

//OTAA or ABP/
bool overTheAirActivation = LORAWAN_NETMODE;

//ADR enable/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;
//funcion de lectura de bateria del nodo
int mapBatteryVoltageToLevel(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
  delay(100);
  byte error, address;
  int nDevices;
  int humedad;
  uint8_t batteryLevel;
  for(address = 10; address < 25; address++ ) {
    Wire.beginTransmission(address);
    delay(300);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.print("  !");
      Serial.println("");    
      
      I2CSoilMoistureSensor sensor(address);
      sensor.begin();
      delay(1000);
      while (sensor.isBusy()) delay(50);
      Serial.print("Soil Moisture Capacitance: ");
      Serial.println(sensor.getCapacitance());
      humedad = sensor.getCapacitance();
      humedad = (1.225)*humedad -(301.25); //formula para escalar valores del sensor a lora a su respectivo rango
      Serial.print("humedad: ");
      Serial.println(humedad);
      sensor.sleep();
      nDevices++;
      Serial.println("");
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
    delay(100);
  }
  //llama la funcion para medir la bateria
  uint16_t voltage = getBatteryVoltage();
  Serial.print("voltage: ");
  Serial.println(voltage);
  batteryLevel = mapBatteryVoltageToLevel(voltage, 3400, 4200, 0, 100);
  Serial.print("batteryLevel: ");
  Serial.println(batteryLevel);

  delay(100);
    appDataSize = 2;
    appData[0] = humedad; //transforma el valor DEC a HEX para enviar a TTN
    appData[1] = batteryLevel; //transforma el valor DEC a HEX para enviar a TTN

}

//downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
  uint32_t color=mcpsIndication->Buffer[0]<<16|mcpsIndication->Buffer[1]<<8|mcpsIndication->Buffer[2];
#if(LoraWan_RGB==1)
  turnOnRGB(color,5000);
  turnOffRGB();
#endif
}

void setup() {
  delay(100);
  Wire.begin();
	Serial.begin(115200);
#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}