#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

uint8_t USB_ReceiveBuff[23];

void USB_Init(void)
{
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
}

void USB_Send(uint8_t *Data,uint32_t Len)
{
	VCP_fops.pIf_DataTx(Data,Len);
}

void USB_Receive(uint8_t Data,uint32_t Index)
{
	USB_ReceiveBuff[Index]=Data;
}
