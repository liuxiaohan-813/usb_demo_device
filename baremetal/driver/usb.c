#include "usb.h"

/* ======================= FIFORead FIFOWrite ===================================== */
#define USB_DMA_EN

void dma_cfg(uint8_t dma_ch, uint8_t ep, uint8_t dir, uint32_t start_addr, uint32_t trans_count)
{
    uint32_t dma_int_st;
    /* config dma start addr and trans num */
    _REG32(USB_COM_CFG_BASE, 0x504 + 0x20 * dma_ch) = start_addr;
    _REG32(USB_COM_CFG_BASE, 0x50C + 0x20 * dma_ch) = trans_count;

    /* disable dma int, band ep and config dir */
    _REG32(USB_COM_CFG_BASE, 0x514 + 0x20 * dma_ch) = 0; // disable dma int
    _REG32(USB_COM_CFG_BASE, 0x500 + 0x20 * dma_ch) = 0x109 | (ep << 4) | (dir << 1);

    /* wait for trans complete */
    do {
        dma_int_st = _REG32(USB_COM_CFG_BASE, 0x510 + 0x20 * dma_ch);
    } while (dma_int_st & BIT(5) != BIT(5));
}

void FIFORead(int nEP, int nBytes, void * pDst)
{
    if (nBytes != 0) {
#ifndef USB_DMA_EN
        int nCount = nBytes;
        BYTE *pby = (BYTE *)pDst;
        int nAddr;

        nAddr = USB_COM_CFG_BASE + 0x100 + nEP * 0x40;
        pby = (BYTE *)pDst;

        while (nCount) {
            *pby++ = REG8(nAddr);
            nCount--;
        }
#else
        /* channel: 0, dir: dma_write(dma read from usb fifo and write to buffer) */
        dma_cfg(0, nEP, DMA_WRITE, (uint32_t)pDst, nBytes);
#endif
    }
    /* CPU clr RxPktRdy */
    REG32(USB_COM_CFG_BASE + 0x108 + nEP * 0x40) &= ~BIT(30);
    return;
}

void FIFOWrite(int nEP, int nBytes, void * pSrc)
{
    if (nBytes != 0) {
#ifndef USB_DMA_EN
        int nCount = nBytes;
        BYTE *pby = (BYTE *)pSrc;
        int nAddr;

        nAddr = USB_COM_CFG_BASE + 0x100 + nEP * 0x40;
        pby = (BYTE *)pSrc;

        while (nCount) {
            REG8(nAddr) = *pby++;
            nCount--;
        }
#else
        /* channel: 1, dir: dma_read(dma read from buffer and write to usb fifo) */
        dma_cfg(1, nEP, DMA_READ, (uint32_t)pSrc, nBytes);
#endif
    }
    /* CPU set TxPktRdy set, CPU set this bit after loading packet into the FIFO */
    REG32(USB_COM_CFG_BASE + 0x108 + nEP * 0x40) |= BIT(31);
    /* wait for send complete */
    while ((REG32(USB_COM_CFG_BASE + 0x108 + nEP * 0x40) & BIT(31)) == BIT(31));
    return;
}
/* =========================================================================== */

void USB_SetDevAddress(uint8_t address)
{
    REG32(USB_COM_CFG_BASE + 0x4) &= ~BITS(8, 14);
    REG32(USB_COM_CFG_BASE + 0x4) |= address << 8;
}

void UsbEp0Up(void)
{
    uint8_t len;

    if (mVarSetupLength > EP0_MAX_LEN) {
        len = EP0_MAX_LEN;
        mVarSetupLength -= len;
    } else {
        len = mVarSetupLength;
        mVarSetupLength = 0;
        setUpStage = 0;
    }
    FIFOWrite(0, len, (void *)VarSetupDescr);
    VarSetupDescr += len;
}

void UsbBusReset(void)
{
    REG32(USB_COM_CFG_BASE + 0x4) |= BIT(1);
    REG32(USB_COM_CFG_BASE + 0x4) &= ~BITS(8, 14); /* FuncAddr = 0 */
}

/* Endpoint Interrupt Status */
uint32_t USB_ReadDevAllEpInterrupt(void)
{
    uint32_t val_0x8;

    val_0x8 = REG32(USB_COM_CFG_BASE + 0x8);

    return val_0x8;
}

uint32_t USB_ReadEPDetailedIntStatus(uint32_t epnum)
{
    uint32_t val_0x128;

    val_0x128 = REG32(USB_COM_CFG_BASE + 0x128 + 0x40 * epnum); // EP Detailed Interrupt Status

    return val_0x128;
}

/* host or device mode */
void USB_SetCurrentMode(USB_OTG_ModeTypeDef mode)
{
    if (mode == USB_DEVICE_MODE) {
        REG32(USB_COM_CFG_BASE + 0x04) &= ~BIT(0); /* controller act as a dev */
        REG32(USB_COM_CFG_BASE + 0xA4) |= BIT(1);  /* phy dev mode */
    } else if (mode == USB_HOST_MODE) {
        REG32(USB_COM_CFG_BASE + 0x04) |= BIT(0);  /* controller act as a host */
        REG32(USB_COM_CFG_BASE + 0xA4) &= ~BIT(1); /* phy host mode */
    }
}

static uint8_t g_speed_mode = USB_OTG_SPEED_FULL;
/* high speed or full speed */
void USB_SetDevSpeed(USB_Speed speed)
{
    if (speed == USB_OTG_SPEED_FULL) {
        REG32(USB_COM_CFG_BASE + 0xA0) &= ~BIT(2);
        REG32(USB_COM_CFG_BASE + 0xD0) |= 59785 << 16 | 59999; // FS
    } else if (speed == USB_OTG_SPEED_HIGH) {
        REG32(USB_COM_CFG_BASE + 0xA0) |= BIT(2);
        REG32(USB_COM_CFG_BASE + 0xD0) |= 7489 << 16 | 7499; // HS
    }
    g_speed_mode = speed;
}

void USB_SetDeviceConnectTime(void)
{
    REG32(USB_COM_CFG_BASE + 0x84) &= ~BITS(8, 15);
    REG32(USB_COM_CFG_BASE + 0x84) |= BIT(8); // The time to check device connect x 2^16
}

static uint32_t FIFOStartAddr = 0;
uint32_t ref_log2(uint32_t fifosize)
{
    uint32_t res = 0;
    while (fifosize > 1) {
        fifosize = fifosize >> 1;
        res++;
    }
    return res;
}

void USB_ActivateEndpoint(uint32_t epnum, uint32_t is_in, uint32_t fifosize)
{
    uint32_t fifosize_log2;

    if (fifosize < 8)
        fifosize = 8;

    fifosize_log2 = ref_log2(fifosize);
    if (epnum == 0) {
        REG32(USB_COM_CFG_BASE + 0x110) |=  0x40 << 16; // Maximum payload, low: 8 full: 8, 16, 32, 64 high: 64
        REG32(USB_COM_CFG_BASE + 0x104) = BIT(31) | FIFOStartAddr << 4 | fifosize_log2 << 0;
    } else {
        if (is_in == USB_EP_IN) {
            REG32(USB_COM_CFG_BASE + 0x110 + 0x40 * epnum) &= ~BIT(31); // Tx
        } else {
            REG32(USB_COM_CFG_BASE + 0x110 + 0x40 * epnum) |= BIT(31); // Rx
        }
        REG32(USB_COM_CFG_BASE + 0x104 + 0x40 * epnum) = BIT(31) | FIFOStartAddr << 4 | fifosize_log2 << 0;
        /* TxRxMaxPayload mode: interrupt */
        if (g_speed_mode = USB_OTG_SPEED_FULL) {
            REG32(USB_COM_CFG_BASE + 0x118 + 0x40 * epnum) = 0x40;
        } else if (g_speed_mode = USB_OTG_SPEED_HIGH) {
            REG32(USB_COM_CFG_BASE + 0x118 + 0x40 * epnum) = 0x400;
        }
    }
    FIFOStartAddr += fifosize / 8; /* 8 bits unit */
}

void USB_PhyInit(void)
{
    /* USB_OTG_ULPI_PHY */
    REG32(USB_COM_CFG_BASE + 0xA0) &= ~BIT(7); // select outside usb2.0 phy
}

void HAL_PCD_DevConnect(void)
{
    REG32(USB_COM_CFG_BASE + 0xA0) |= BIT(4); // the transfer will normal going
}
