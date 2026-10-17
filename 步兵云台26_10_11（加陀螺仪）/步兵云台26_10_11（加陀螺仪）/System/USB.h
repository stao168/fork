#ifndef __USB_H
#define __USB_H

#include <stdint.h>

extern uint8_t USB_ReceiveBuff[];

void USB_Init(void);
void USB_Send(uint8_t *Data,uint32_t Len);
void USB_Receive(uint8_t Data,uint32_t Index);

#endif
