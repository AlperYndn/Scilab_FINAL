#include <MQUnifiedsensor.h>
#include <Adafruit_Sensor.h>
#include <SFE_BMP180.h>
#include "DHT.h"
#include <Wire.h>

#define         MQ4_Pin                 (A4)
#define         MQ135_Pin               (A3)
#define         Board                   ("Arduino MEGA") 
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) 
#define         RatioMQ4CleanAir        (4.4) //RS / R0 = 60 ppm
#define         RatioMQ135CleanAir      (3.6) //RS / R0 = 3.6 ppm
#define         DHTTYPE                 (DHT11)
#define         DHTPIN                  (22)
#define         ALTITUDE                1655.0

MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ4_Pin, "MQ-4");
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ135_Pin, "MQ-135");
DHT dht(DHTPIN, DHTTYPE);
SFE_BMP180 bmp180;

const int underground_temperature_const = 5.0044;

void setup() {
  Serial.begin(115200);

  dht.begin();
  bmp180.begin();

  MQ4.setRegressionMethod(1);
  MQ135.setRegressionMethod(1);

  MQ4.init();
  MQ135.init();

  float calcR0_MQ4 = 0; float calcR0_MQ135 = 0;
  
  for(int i = 1; i<=20; i ++){
    MQ4.update();
    calcR0_MQ4 += MQ4.calibrate(RatioMQ4CleanAir);

    MQ135.update();
    calcR0_MQ135 += MQ135.calibrate(RatioMQ135CleanAir);
  }

  MQ4.setR0(calcR0_MQ4/20);
  MQ135.setR0(calcR0_MQ135/20);
}

void loop() {
  
  char status;
  double T,P;
  float underground_humadity_data, underground_humadity,
        underground_temperature_data, underground_temperature_ohm, underground_temperature;

  underground_temperature_data = analogRead(A0);
  underground_temperature_ohm = underground_temperature_data/underground_temperature_data;
  underground_temperature = (underground_temperature_ohm - 100)/0.3944;

  underground_humadity_data = analogRead(A0) - 223.00;
  underground_humadity = 100 - (underground_humadity_data*100/800);
   
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  MQ4.update();MQ135.update();
  MQ4.setA(1012.7); MQ4.setB(-2.786);float CH4 = MQ4.readSensor(); 
  MQ135.setA(110.47); MQ135.setB(-2.862);float CO2 = MQ135.readSensor(); 
  MQ135.setA(102.2 ); MQ135.setB(-2.473);float NH4 = MQ135.readSensor();

  static float pressure;

  status = bmp180.startTemperature();
  if (status != 0) {
    status = bmp180.getTemperature(T);
    if (status != 0) {
      status = bmp180.startPressure(3);
      if (status != 0) {
        status = bmp180.getPressure(P, T);
        if (status != 0) {
          pressure = P; // hPa 
        }
      }
    }
  }

float txData[8]= {CH4, NH4, CO2, pressure, temperature, humidity, 
                  underground_temperature, underground_humadity};
  String txString = String("A" + String(txData[0]) + "BC" + String(txData[1]) 
                            + "DE" + String(txData[2]) + "FG" + String(txData[3]) 
                            + "HI" + String(txData[4]) + "JK" + String(txData[5]) 
                            + "L" + String(txData[5]) + "M" + String(txData[6]) 
                            + "NO" + String(txData[7]) + "P" );
                            
  Serial.println(txString);
  delay(1000);
}
