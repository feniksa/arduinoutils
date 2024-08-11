#include <SoftwareSerial.h>
#include <util/crc16.h>

#define RXD0 2	// we use PIN2 of Arduino UNO
#define TXD0 3	// we use PIN3 of Arduino UNO

SoftwareSerial serial(RXD0, TXD0);

byte ModReadBuffer[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
byte BufferValue[25];

// from https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
uint16_t crc16_update(uint16_t crc, byte a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
	  if (crc & 1)
		crc = (crc >> 1) ^ 0xA001;
	  else
		crc = (crc >> 1);
  }
  return crc;
}

uint16_t crc16_ibm(byte* data, size_t size, uint16_t crc)
{  
  for (size_t i = 0; i < size; ++i)
	//crc = crc16_update(crc, data[i]);
	crc = _crc16_update(crc, data[i]);
  return crc; 
}

void setup() {
  Serial.begin(9600);
  while(!Serial) {}
  
  serial.begin(9600);
}


void loop() {	
  Serial.println("calculate cr16");
  
  uint16_t crc2 = crc16_ibm(ModReadBuffer, sizeof(ModReadBuffer)-2, 0xffff);
  
  Serial.print("crc2 is ");
  Serial.print(crc2);
  Serial.print(" ");
  Serial.print(crc2, HEX);
  Serial.println("");

  uint8_t crc_low = crc2 & 0x00FF;
  uint8_t crc_high = (crc2 & 0xFF00) >> 8;
  
  ModReadBuffer[6] = crc_low;
  ModReadBuffer[7] = crc_high;

  for (int i = 0; i < sizeof(ModReadBuffer); ++i) {
	Serial.print("0x");
	Serial.print(ModReadBuffer[i], HEX);
	Serial.print(" ");
  }
  Serial.println("");	   
	
  //serial.write(ModReadBuffer, sizeof(ModReadBuffer)); 
  for (byte i = 0; i < sizeof(ModReadBuffer); ++i) {   
	serial.write(ModReadBuffer[i]);				  
  }  

  Serial.println("wait for remote chip");
  int counter = 0;
  while (!serial.available()) {
	++counter;
	if (counter == 100) {
	  Serial.println("no reply, restart");
	  return;
	}	 
	delay(10);
  }  

  Serial.println("read data from serial");
  for(byte i=0;i<sizeof(BufferValue); ++i){
	  BufferValue[i] = serial.read();
	  Serial.print("0x");
	  Serial.print(BufferValue[i], HEX);	  
	  Serial.print(" "); 
	}	  
	Serial.println("");

  Serial.print("check crc ");

  size_t offset = sizeof(BufferValue) - 2;
  
  uint8_t high = BufferValue[offset + 0];
  uint8_t low  = BufferValue[offset + 1];

  Serial.print(high, HEX);
  Serial.print(" ");
  Serial.print(low, HEX);  
  Serial.print(" ");
  
  uint16_t crc_to_check = (high << 8) | low;
  uint16_t calculated_crc = crc_to_check = crc16_ibm(BufferValue, sizeof(BufferValue)-2, 0xffff);

  if (crc_to_check == calculated_crc) {
	Serial.println("OK");
  } else {
	Serial.println("FAIL");
	delay(100);  
	return;
  }

  // decode data
  uint8_t slaveId = BufferValue[0];
  uint8_t dataSize = BufferValue[2];

  offset = 3;
  uint16_t voltage = (BufferValue[offset++] << 8) | BufferValue[offset++];
  uint16_t current = (BufferValue[offset++] << 8) | BufferValue[offset++];	
  uint16_t power = (BufferValue[offset++] << 8) | BufferValue[offset++];	
  uint16_t energy = (BufferValue[offset++] << 8) | BufferValue[offset++];	 
  uint16_t frequency = (BufferValue[offset++] << 8) | BufferValue[offset++];	
  uint16_t power_factor = (BufferValue[offset++] << 8) | BufferValue[offset++];    
  uint16_t alarm = (BufferValue[offset++] << 8) | BufferValue[offset++];	
  

  Serial.print("voltage ");
  Serial.print((float)voltage / 10.0f);
  Serial.print(" v, current ");
  Serial.print((float)current / 1000.0f);
  Serial.print(" A, power ");
  Serial.print((float)power / 10.0f);

  Serial.print(" W, energy ");
  Serial.print((float)energy / 10.0f);
  Serial.print(" Wh, frequency ");
  Serial.print((float)frequency / 10.0f);
  Serial.print(" Hz, power factor ");
  Serial.print((float)power_factor / 100.0f);
  Serial.print(", alarm status ");
  if (alarm == 0) {
	Serial.print(" ok ");
  } else {
	Serial.print(" ALARM! ");
  } 
  
  delay(5000);	
}
