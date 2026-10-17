#ifndef __REFEREESYSTEM_CRCTABLE_H
#define __REFEREESYSTEM_CRCTABLE_H

//crc8 generator polynomial:G(x)=x8+x5+x4+1//CRC8-MAXIM
extern uint8_t CRC8_Initial;//CRC8初始值
extern const uint8_t CRC8_Table[256];

//crc16 generator polynomial:G(x)=x16+x12+x5+1//CRC16-CCITT
extern uint16_t CRC16_Initial;//CRC16初始值
extern const uint16_t CRC16_Table[256];

#endif
