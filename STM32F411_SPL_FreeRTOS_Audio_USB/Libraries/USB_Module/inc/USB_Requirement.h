#ifndef __USB_REQUIREMENT_H__
#define __USB_REQUIREMENT_H__

#include "usbh_core.h"
#include "ff.h"
#include <stdio.h>
#include "main.h"

HOST_State USB_Reject(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost);
void write_data(uint8_t wtext[], uint32_t bytewritten, USBH_Status status);


#endif
