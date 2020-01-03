#include <DeepSleepScheduler.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "BME280.h"

#define WEATHER_1 0x1

RH_RF95 rf95;
static const uint8_t node_id = 1; // LoRa End Node ID, should be eeprom
static const float frequency = 868.0;
volatile unsigned int count = 1;
BlueDot_BME280 bme;

float pressure;
float temperature;
float humidity;
uint16_t voltage;

struct Measurement
{
  uint8_t node_id;
  uint8_t data_type;
  float pressure;
  float temperature;
  float humidity;
  uint16_t voltage;
  unsigned int count;
};

Measurement measurement;

void measureAndSend() {
  Serial.println(F("Measure and send"));
  
  if (!rf95.init())
    Serial.println("init failed");
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);
  rf95.setSyncWord(0x34);

  voltage = readVcc();

  uint8_t bme_result = bme.init();
  
  if (bme_result != 96) {
    Serial.print("BME failed.");
  }

  Serial.print("LoRa Weather module ID: ");
  Serial.print(node_id, HEX);  
  Serial.println();
  Serial.print("COUNT=");
  Serial.print(count);
  Serial.println("    ###########");
  count++;

  read_bme();
  char data[50] = {0};
  int dataLength;

  measurement.node_id = node_id;
  measurement.data_type = WEATHER_1;
  measurement.humidity = humidity;
  measurement.temperature = temperature;
  measurement.pressure = pressure;
  measurement.voltage = voltage;
  measurement.count = count;

  memcpy(data, &measurement, sizeof(measurement));  
  dataLength = sizeof(measurement);

  Serial.print("Data: ");
  Serial.println(dataLength);
  Serial.println(data);
  
  uint16_t crcData = CRC16((unsigned char*)data, dataLength); //get CRC DATA
  
  unsigned char sendBuf[50] = {0};
  for (int i = 0; i < dataLength; i++)
  {
    sendBuf[i] = data[i];
  }

  sendBuf[dataLength] = (unsigned char)crcData; // Add CRC to LoRa Data
  sendBuf[dataLength + 1] = (unsigned char)(crcData >> 8); // Add CRC to LoRa Data

  Serial.print(F("Data to be sent(with CRC):    "));
  for (int i = 0; i < (dataLength + 2); i++)
  {
    Serial.print(sendBuf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  rf95.send(sendBuf, dataLength + 2); //Send LoRa Data

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//Reply data array
  uint8_t len = sizeof(buf);//reply data length

  if (rf95.waitAvailableTimeout(3000))// Check If there is reply in 3 seconds.
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))//check if reply message is correct
    {
      if (buf[0] == node_id) // Check if reply message has the our node ID
      {
        
        Serial.print("Got Reply from Gateway: ");//print reply
        Serial.println((char*)buf);        
      }
    }
    else
    {
      Serial.println(F("recv failed"));
      rf95.send(sendBuf, strlen((char*)sendBuf)); //resend if no reply
    }
  }
  else
  {
    Serial.println(F("No reply, is LoRa gateway running?")); //No signal reply
    rf95.send(sendBuf, strlen((char*)sendBuf));//resend data
  }
  
  // go to sleep
  rf95.sleep();
  delay(200);
  scheduler.scheduleDelayed(measureAndSend, 600000L);
}


void setup()
{
  Serial.begin(9600);
  scheduler.schedule(measureAndSend);
}


uint16_t calcByte(uint16_t crc, uint8_t b)
{
  uint32_t i;
  crc = crc ^ (uint32_t)b << 8;

  for ( i = 0; i < 8; i++)
  {
    if ((crc & 0x8000) == 0x8000)
      crc = crc << 1 ^ 0x1021;
    else
      crc = crc << 1;
  }
  return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer, uint32_t length)
{
  uint16_t wCRC16 = 0;
  uint32_t i;
  if (( pBuffer == 0 ) || ( length == 0 ))
  {
    return 0;
  }
  for ( i = 0; i < length; i++)
  {
    wCRC16 = calcByte(wCRC16, pBuffer[i]);
  }
  return wCRC16;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // TODO: calibrate: internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
  return result; // Vcc in millivolts
}


void read_bme() {
  delay(50);
  pressure = (float) (bme.readPressure());
  delay(20);
  humidity = (float) (bme.readHumidity());
  delay(20);
  temperature = (float) bme.readTempC();
  delay(20);
}

void loop()
{
  scheduler.execute();

}
