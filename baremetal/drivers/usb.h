#include "utils.h"

#ifndef _USB_H
#define _USB_H

#define USB_COM_CFG_BASE                    0x10020000

#define DMA_WRITE  0
#define DMA_READ   1

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

void USB_PhyInit(void);
void HAL_PCD_DevConnect(void);
void USB_ActivateEndpoint(uint32_t epnum, uint32_t is_in, uint32_t fifosize);
uint32_t ref_log2(uint32_t fifosize);
void USB_SetDeviceConnectTime(void);
void USB_SetDevSpeed(USB_Speed speed);
void USB_SetCurrentMode(USB_OTG_ModeTypeDef mode);
uint32_t USB_ReadEPDetailedIntStatus(uint32_t epnum);
uint32_t USB_ReadDevAllEpInterrupt(void);
void UsbBusReset(void);


void dma_cfg(uint8_t dma_ch, uint8_t ep, uint8_t dir, uint32_t start_addr, uint32_t trans_count);
void FIFORead(int nEP, int nBytes, void * pDst);
void FIFOWrite(int nEP, int nBytes, void * pSrc);

void USB_SetDevAddress(uint8_t address);

void Ep1SendData(void);
void ProcScsiCommand(void);
void ProcScsiOutData(void);

#endif
