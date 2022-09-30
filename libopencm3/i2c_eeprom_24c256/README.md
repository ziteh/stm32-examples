# I2C EEPROM 24C256

## USART Command
- Write
  - command: `0x00 <address_1> <address_2> <data>`.
  - return: `0xF0`.
- Read
  - Command: `0x01 <address_1> <address_2>`.
  - return: `<data>` 

For example:
```c
>> 0x00 0x00 0x00 0xAB  // Write 0xAB to EEPROM address 0x0000.
<< 0xF0                 // Return from STM32.

>> 0x00 0x00 0x01 0xCD  // Write 0xCD to EEPROM address 0x0001.
<< 0xF0                 // Return from STM32.

>> 0x01 0x00 0x00       // Read EEPROM address 0x0000.
<< 0xAB                 // Data readed.

>> 0x01 0x00 0x01       // Read EEPROM address 0x0001.
<< 0xCD                 // Data readed.

>> 0x00 0x00 0x01 0x39  // Write 0x39 to EEPROM address 0x0001.
<< 0xF0                 // Return from STM32.

>> 0x01 0x00 0x01       // Read EEPROM address 0x0001.
<< 0x39                 // Data readed.
```

> USART params: 
> - Baud rate: 9600
> - Data bits: 8
> - Parity: None
> - Stop bits: 1
