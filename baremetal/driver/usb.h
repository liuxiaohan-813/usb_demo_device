#include "utils.h"

#ifndef _USB_H
#define _USB_H

#define USB_COM_CFG_BASE                    0x10020000

typedef enum {
    USB_OTG_SPEED_FULL = 0U,
    USB_OTG_SPEED_HIGH = 1U,
    USB_OTG_SPEED_LOW = 2U,
} USB_Speed;

enum Epdir {
    USB_EP_OUT = 0,
    USB_EP_IN = 1,
    USB_EP0 = 2,
};

typedef enum {
    USB_DEVICE_MODE  = 0,
    USB_HOST_MODE    = 1,
    USB_DRD_MODE     = 2
} USB_OTG_ModeTypeDef;

#ifndef BYTE
typedef unsigned char BYTE;
#endif
#ifndef WORD
typedef unsigned short WORD;
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))

void FIFORead(int nEP, int nBytes, void * pDst);
void FIFOWrite(int nEP, int nBytes, void * pSrc);

void USB_SetDevAddress(uint8_t address);

void Ep1SendData(void);
void ProcScsiCommand(void);
void ProcScsiOutData(void);

#endif
