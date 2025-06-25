/*
 * According to datasheet https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
 * on section "8.1.1 Automatic microcontroller interface detection" to use the
 * UART interface pins EA (32) and I2C (1) must be connected to GND.
 * The board I use comes with SPI interface selected so I had to manually connect pin
 * EA to ground.
 */

#include <Arduino.h>
#include <MFRC522.h>

MFRC522_UART::MFRC522_UART(HardwareSerial &serial, uint8_t resetPin)
	: _resetPowerDownPin(resetPin), _serialPort(serial) {}

bool MFRC522_UART::PCD_Init() {
  _serialPort.begin(9600, SERIAL_8N1);

  if (_resetPowerDownPin != UNUSED_PIN) {
    pinMode(_resetPowerDownPin, OUTPUT);
    digitalWrite(_resetPowerDownPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_resetPowerDownPin, HIGH);
    delay(50);
    return true;
  }
  return false;
}

void MFRC522_UART::PCD_WriteRegister(MFRC522::PCD_Register reg, uint8_t value) {
	// Mask used for writing to registers. See section "8.1.3.3 UART framing" from datasheet.
	_serialPort.write(0x7F & reg);
	// Waits for confirmation
  	while (!_serialPort.available());
	// Removes confirmation uint8_t from input buffer
  	_serialPort.read();
	// Writes data
	_serialPort.write(value);
}

void MFRC522_UART::PCD_WriteRegister(
  	MFRC522::PCD_Register reg, uint8_t count, uint8_t *values) {

	for (uint8_t index = 0; index < count; ++index)
    	this->PCD_WriteRegister(reg, values[index]);
}

uint8_t MFRC522_UART::PCD_ReadRegister(MFRC522::PCD_Register reg) {
	// Mask used for reading from registers. See section "8.1.3.3 UART framing" from datasheet.
	_serialPort.write(0x80 | reg);
	// Waits for input
	while (!_serialPort.available());
	// Reads data
	uint8_t value = _serialPort.read();
	return value;
}

void MFRC522_UART::PCD_ReadRegister(
  	MFRC522::PCD_Register reg, uint8_t count, uint8_t *values, uint8_t rxAlign) {

	uint8_t index = 0;
	// Only update bit positions rxAlign..7 in values[0]
	if (rxAlign) {
		uint8_t mask = 0xFF << rxAlign;
		uint8_t value = this->PCD_ReadRegister(reg);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		++index;
	}

	while (index < count) {
		// Keeps reading from the same address
		values[index] = this->PCD_ReadRegister(reg);
		++index;
	}
}