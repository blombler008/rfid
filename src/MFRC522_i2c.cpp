#include <MFRC522.h>

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a uint8_t to the specified register in the MFRC522_I2C chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_I2C::PCD_WriteRegister(        MFRC522::PCD_Register reg,               ///< The register to write to. One of the PCD_Register enums.
                                            uint8_t value              ///< The value to write.
) {
        _wire.beginTransmission(_chipAddress);
        _wire.write(reg);
        _wire.write(value);
        _wire.endTransmission();
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522_I2C chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_I2C::PCD_WriteRegister(        MFRC522::PCD_Register reg,               ///< The register to write to. One of the PCD_Register enums.
                                                                        uint8_t count,             ///< The number of bytes to write to the register
                                                                        uint8_t *values    ///< The values to write. uint8_t array.
                                                                ) {
        _wire.beginTransmission(_chipAddress);
        _wire.write(reg);
        for (uint8_t index = 0; index < count; index++) {
                _wire.write(values[index]);
        }
        _wire.endTransmission();
} // End PCD_WriteRegister()

/**
 * Reads a uint8_t from the specified register in the MFRC522_I2C chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t MFRC522_I2C::PCD_ReadRegister( MFRC522::PCD_Register reg        ///< The register to read from. One of the PCD_Register enums.
                                                                ) {
        uint8_t value;
        //digitalWrite(_chipSelectPin, LOW);                    // Select slave
        _wire.beginTransmission(_chipAddress);
        _wire.write(reg);
        _wire.endTransmission();

        _wire.requestFrom(_chipAddress, (uint8_t)1 /* bytes to request */);
        value = _wire.read();

        return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522_I2C chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_I2C::PCD_ReadRegister( MFRC522::PCD_Register reg,               ///< The register to read from. One of the PCD_Register enums.
                                                                uint8_t count,             ///< The number of bytes to read
                                                                uint8_t *values,   ///< uint8_t array to store the values in.
                                                                uint8_t rxAlign    ///< Only bit positions rxAlign..7 in values[0] are updated.
                                                                ) {
        if (count == 0) {
                return;
        }
        uint8_t address = reg;
        uint8_t index = 0;                                                 // Index in values array.
        _wire.beginTransmission(_chipAddress);
        _wire.write(address);
        _wire.endTransmission();
        _wire.requestFrom(_chipAddress, count);
        while (_wire.available()) {
                if (index == 0 && rxAlign) {            // Only update bit positions rxAlign..7 in values[0]
                        // Create bit mask for bit positions rxAlign..7
                        uint8_t mask = 0;
                        for (uint8_t i = rxAlign; i <= 7; i++) {
                                mask |= (1 << i);
                        }
                        // Read value and tell that we want to read the same address again.
                        uint8_t value = _wire.read();
                        // Apply mask to both current value of values[0] and the new data in value.
                        values[0] = (values[index] & ~mask) | (value & mask);
                }
                else { // Normal case
                        values[index] = _wire.read();
                }
                index++;
        }
} // End PCD_ReadRegister()


bool MFRC522_I2C::PCD_Init() {
	_wire.begin();
        // Set the chipSelectPin as digital output, do not select the slave yet

        // Set the resetPowerDownPin as digital output, do not reset or power down.
        pinMode(_resetPowerDownPin, OUTPUT);

        if (digitalRead(_resetPowerDownPin) == LOW) {   //The MFRC522 chip is in power down mode.
                digitalWrite(_resetPowerDownPin, HIGH);         // Exit power down mode. This triggers a hard reset.
                // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
                delay(50);
		return true;
        }
	return false;
};

