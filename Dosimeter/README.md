# UV Dosimeter
BLE Application intended to run on a Taiyo Yuden EYSHSNZWZ module.
Utilizes the nRF SDK v17.0.0

Application periodically samples the analog voltage of a pin, mutates this reading into usable values, and then writes to the included UV BLE service.

## Environmental Service:
UV Index Char - Contains the current UV Index value

UV Dosage Char - Contains the user's daily SED (Currently not resettable)

UV TX Char - On long press of the button, the EEPROM is read and all values are added to this char.

