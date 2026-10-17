#ifndef __DEBUGPORT_H
#define __DEBUGPORT_H

#include "stdint.h"

void DebugPort_Init(void);
void DebugPort_SendArray(uint8_t *Array,uint16_t Length);
void DebugPort_SendString(char *String);
void DebugPort_Printf(char *format,...);

#endif
