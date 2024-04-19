#include "USB_Requirement.h"


extern USBH_HOST			USB_Host;
FIL 									MyFile;

HOST_State USB_Reject(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost)
{
	return HOST_SUSPENDED;
}	

void write_data(uint8_t wtext[], uint32_t bytewritten, USBH_Status status)
{
	FATFS fs;
	FRESULT res;
	if(f_mount(bytewritten, &fs) != FR_OK)
	{
		USBH_ErrorHandle(&USB_Host, status); 
	}
	else
	{
		/*Create and open new file USB.TXT*/
		if(f_open(&MyFile, "USB.TXT", FA_CREATE_ALWAYS|FA_CREATE_NEW) != FR_OK)
		{
			USBH_ErrorHandle(&USB_Host, status);
		}
		/*Write into file USB.TXT*/
		else
		{
			res = f_write(&MyFile, wtext, sizeof(wtext), (void *)bytewritten);
			if((bytewritten == 0) | (res != FR_OK))
			{
				USBH_ErrorHandle(&USB_Host, status);
			}
			else
			{
				/* Close an open file object */
				 f_close(&MyFile);								
			}
		}
	}
}

void f_opendir_scan(void)
{
	

}

