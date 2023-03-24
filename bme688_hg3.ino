#include <RadioHead.h>

/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#define INT 2 //GP2
#define CS 4

#define CLIENT_ADDRESS 10
#define SERVER_ADDRESS 2
#define RFM95_RST 9
#define PIN_CS 10
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK


//Singleton instance of the radio driver
RH_RF95 rf95(CS, INT);

RHReliableDatagram manager(rf95, CLIENT_ADDRESS);

Bme68x bme;

/**
 * @brief Initializes the sensor and hardware settings
 */
void setup(void)
{
  uint32_t unique_id;
  SPI.begin();
  
  Serial.begin(115200);
  pinMode(INT, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(INT), read_packets, CHANGE);
  //Serial.println("Starting rf95 & bme drivers...");
	while (!Serial)
		delay(10);

   // initializes rf95 driver
  while(!manager.init()) {
    Serial.println("RF95 init failed");  
  }
  // // set freq & tx power
  // if (!rf95.setFrequency(915.0)) {
  //   Serial.println("Set frequency failed!");

  // }
  // rf95.setTxPower(23, false);

	/* initializes the sensor based on I2C library */
	bme.begin(PIN_CS, SPI);
	if(bme.checkStatus()) {
		if (bme.checkStatus() == BME68X_ERROR) {
			Serial.println("Sensor error:" + bme.statusString());
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING) {
			Serial.println(bme.statusString());
		}

	}
  unique_id = bme.getUniqueId();
  Serial.println("Sensor ID: " + unique_id);
	/* Set the default configuration for temperature, pressure and humidity */
	bme.setTPH();

	/* Set the heater configuration to 300 deg C for 100ms for Forced mode */
	bme.setHeaterProf(300, 100);


}
void loop(void)
{
	bme68xData data;
  bool valid_d = true;
  uint8_t nFieldsLeft = 0;
	bme.setOpMode(BME68X_PARALLEL_MODE);
	delayMicroseconds(bme.getMeasDur());
  
	 if (bme.fetchData()) {

    do{
		  nFieldsLeft = bme.getData(data);
      Serial.print(String(millis()) + ", ");
      // Serial.print(String(data.temperature) + ", ");
      // Serial.print(String(data.pressure) + ", ");
      // Serial.print(String(data.humidity) + ", ");
      Serial.print(String(data.gas_resistance) + ", ");
      Serial.println(data.status, HEX);
	  }
    while (nFieldsLeft);
    valid_d = true;
   }
   else {
     valid_d = false;
   }

  if (valid_d == true) {
    Serial.println("Sending BME data..");
    uint8_t bme_data[] = {data.temperature, data.pressure, data.humidity, data.gas_resistance, data.status};

    uint8_t buff_size = sizeof(bme_data);
  
    read_packets(bme_data, buff_size);
  }
  else {
    Serial.println("Invalid data");
  }

  delay(100);

}

void read_packets(uint8_t *inbuff, uint8_t size_buffer) {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  if (manager.sendtoWait(inbuff, size_buffer, SERVER_ADDRESS)) {
    Serial.println("Sending packets..");
  // Now wait for a reply
   uint8_t len = sizeof(buf);

    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else {
      Serial.println("recv failed");
    }
  }
  else {
    Serial.println("No reply, is rf95_server running?");
  }
  delay(100);
}
