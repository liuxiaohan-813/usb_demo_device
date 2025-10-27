#include <stdio.h>
#include "usb_sdk_hal.h"
#include "usb.h"
#include "desc.h"

#if 0

ssize_t _write(int fd, const void* ptr, size_t len)
{

}

#endif

/* ======================= FIFORead FIFOWrite ===================================== */
#define USB_DMA_EN

void dma_cfg(uint8_t dma_ch, uint8_t ep, uint8_t dir)
{
    _REG32(USB_COM_CFG_BASE, 0x514 + 0x20 * dma_ch) = 0; // disable dma int
    _REG32(USB_COM_CFG_BASE, 0x500 + 0x20 * dma_ch) = 0x109 | (ep << 4) | (dir << 1);
}

void dma_write_cfg(uint8_t dma_ch, uint8_t ep, uint32_t start_addr, uint32_t dma_count)
{
    uint32_t dma_int_st;

    _REG32(USB_COM_CFG_BASE, 0x504 + 0x20 * dma_ch) = start_addr;
    _REG32(USB_COM_CFG_BASE, 0x50C + 0x20 * dma_ch) = dma_count;
    dma_cfg(dma_ch, ep, 0);
    dma_int_st = _REG32(USB_COM_CFG_BASE, 0x510 + 0x20 * dma_ch);
    while (dma_int_st & BIT(5) != BIT(5));
}

void dma_read_cfg(uint8_t dma_ch, uint8_t ep, uint32_t start_addr, uint32_t dma_count)
{
    uint32_t dma_int_st;

    _REG32(USB_COM_CFG_BASE, 0x504 + 0x20 * dma_ch) = start_addr;
    _REG32(USB_COM_CFG_BASE, 0x50C + 0x20 * dma_ch) = dma_count;
    dma_cfg(dma_ch, ep, 1);
    dma_int_st = _REG32(USB_COM_CFG_BASE, 0x510 + 0x20 * dma_ch);
    while (dma_int_st & BIT(5) != BIT(5));
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
        dma_write_cfg(0, nEP, (uint32_t)pDst, nBytes);
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
        dma_read_cfg(0, nEP, (uint32_t)pSrc, nBytes);
#endif
    }
    /* CPU set TxPktRdy set, CPU set this bit after loading packet into the FIFO */
    REG32(USB_COM_CFG_BASE + 0x108 + nEP * 0x40) |= BIT(31);
    /* wait for send complete */
    while ((REG32(USB_COM_CFG_BASE + 0x108 + nEP * 0x40) & BIT(31)) == BIT(31));
    return;
}
/* =========================================================================== */

unsigned char mVarSetupRequest;
uint16_t mVarSetupLength;
const unsigned char *VarSetupDescr = NULL;
uint16_t SetupMaxLength = 0;
uint8_t setUpStage = 0;
uint8_t StatusEnd = 0;
uint8_t EP0_MAX_LEN = DevDes[7];

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

uint8_t CBW[31];
uint8_t CSW[13];

void UsbEp1In(void)
{
    Ep1SendData();
}

void UsbEp2Out(void)
{
    uint16_t length;

    length = REG32(USB_COM_CFG_BASE + 0x120 + 0x40 * 2); // Rx FIFO size
    printf("len = %0x\n", length);
    if (length == 31) {
        FIFORead(2, sizeof(CBW), CBW);
        for (int i = 0; i < 16; i++) {
            printf("%0x ", CBW[i]);
        }
       // if ((CBW[0]=='U') && (CBW[1]=='S') && (CBW[2]=='B') && (CBW[3]=='C')) {
        if ((CBW[1]=='S') && (CBW[2]=='B') && (CBW[3]=='C')) {
            ProcScsiCommand();
        }
    } else {
        ProcScsiOutData();
    }
}

typedef	union _REQUEST_PACK {
	unsigned char buffer[8];
	struct {
		unsigned char bmReuestType;
		unsigned char bRequest;
		uint16_t wValue;
		uint16_t wIndx;
		uint16_t wLength;
	} r;
} mREQUEST_PACKET, *mpREQUEST_PACKET;
mREQUEST_PACKET request;

void handle_get_desc_request(mREQUEST_PACKET USB_request)
{
    int len;

    switch (USB_request.r.wValue >> 8) {
        case 1:
            printf("device desc\n");
            VarSetupDescr = DevDes;
            mVarSetupLength = MIN(sizeof(DevDes), SetupMaxLength);
            break;
        case 2:
            printf("cfg desc\n");
            VarSetupDescr = ConDes;
            mVarSetupLength = MIN(sizeof(ConDes), SetupMaxLength);
            break;
        case 3:
            printf("string desc\n");
            if ((USB_request.r.wValue & 0xff) == 0) {
                printf("LangDes\n");
                VarSetupDescr = LangDes;
                mVarSetupLength = MIN(sizeof(LangDes), SetupMaxLength);
            } else if ((USB_request.r.wValue & 0xff) == 1) {
                printf("Manuf_Des\n");
                VarSetupDescr = Manuf_Des;
                mVarSetupLength = MIN(sizeof(Manuf_Des), SetupMaxLength);
            } else if ((USB_request.r.wValue & 0xff) == 2) {
                printf("Prod_Des\n");
                VarSetupDescr = Prod_Des;
                mVarSetupLength = MIN(sizeof(Prod_Des), SetupMaxLength);
            } else if ((USB_request.r.wValue & 0xff) == 3) {
                printf("SerDes\n");
                VarSetupDescr = SerDes;
                mVarSetupLength = MIN(sizeof(SerDes), SetupMaxLength);
            } else {
                printf("other\n");
            }
            break;
        case 4:
            printf("interface desc\n");
            break;
        case 5:
            printf("endpoint desc\n");
            break;
        case 6:
            printf("qualifier desc\n");
            VarSetupDescr = DeviceQualifierDesc;
            mVarSetupLength = MIN(sizeof(DeviceQualifierDesc), SetupMaxLength);
            break;
        default:
            printf("other desc\n");
    }
    UsbEp0Up();
}

void USB_SetDevAddress(uint8_t address)
{
    REG32(USB_COM_CFG_BASE + 0x4) &= ~BITS(8, 14);
    REG32(USB_COM_CFG_BASE + 0x4) |= address << 8;
}

enum Std_bRequest_ID {
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    SET_FEATURE = 3,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12,
};

void handle_std_request(mREQUEST_PACKET USB_request)
{
    switch (USB_request.r.bRequest) {
        case GET_STATUS:
            printf("GET_STATUS->");
            break;
        case CLEAR_FEATURE:
            printf("CLEAR_FEATURE->");
            break;
        case SET_FEATURE:
            printf("SET_FEATURE->");
            break;
        case SET_ADDRESS:
            printf("SET_ADDRESS->\r\n");
            USB_SetDevAddress(USB_request.r.wValue);
            break;
        case GET_DESCRIPTOR:
            printf("GET_DESCRIPTOR->");
            handle_get_desc_request(USB_request);
            break;
        case SET_DESCRIPTOR:
            printf("SET_DESCRIPTOR->");
            break;
        case GET_CONFIGURATION:
            printf("GET_CONFIGURATION->");
            break;
        case SET_CONFIGURATION:
            printf("SET_CONFIGURATION->");
            break;
        case GET_INTERFACE:
            printf("GET_INTERFACE->");
            break;
        case SET_INTERFACE:
            printf("SET_INTERFACE->");
            break;
        case SYNCH_FRAME:
            printf("SYNCH_FRAME->");
            break;
    }
}

enum Class_bRequest_ID {
    GET_MAX_LUN = 0xFE,
    MASS_STORAGE_RESET = 0xFF,
};

void handle_class_request(mREQUEST_PACKET USB_request)
{
    switch (USB_request.r.bRequest) {
        case GET_MAX_LUN:
            printf("get max len\r\n");
            VarSetupDescr = (uint8_t *)MaxLun;
            mVarSetupLength = 1;
            break;
        case MASS_STORAGE_RESET:
            printf("MASS_STORAGE_RESET\r\n");
            mVarSetupLength = 0;
        default:
            printf("error bRequest\r\n");
    }
    UsbEp0Up();
}

void handle_usb_request(mREQUEST_PACKET USB_request)
{
    setUpStage = 1;

    if (USB_request.r.bmReuestType & 0x80) {
        printf("dir: GET\n");
    } else {
        printf("dir: SET\n");
    }
    switch ((request.r.bmReuestType >> 5) & 0x3) {
        case 0:
            printf("std request->");
            handle_std_request(USB_request);
            break;
        case 1:
            printf("class request->");
            handle_class_request(USB_request);
            break;
        case 2:
            printf("manufacture request->");
            break;
        case 3:
            printf("reserve req!\r\n");
            break;
    }
}

void UsbBusReset(void)
{
    REG32(USB_COM_CFG_BASE + 0x4) |= BIT(1);
    REG32(USB_COM_CFG_BASE + 0x4) &= ~BITS(8, 14); /* FuncAddr = 0 */
}

/* Connection Interrupt Status */
uint32_t USB_ReadConnectionInterrupt(void)
{
    uint32_t val_0x10, val_0x14;
    uint32_t status;

    val_0x14 = REG32(USB_COM_CFG_BASE + 0x14); // enable
    val_0x10 = REG32(USB_COM_CFG_BASE + 0x10); // status

    status = val_0x10 & val_0x14;

    if (status & BIT(0)) {
        printf("suspend\n");
    } else if (status & BIT(1)) {
        printf("resume\n");
    } else if (status & BIT(2)) {
        printf("Reset int\n");
        UsbBusReset();
    } else if (status & BIT(4)) {
        printf("SOF int\n");
    } else if (status & BIT(5)) {
        printf("Connect int\n");
    } else if (status & BIT(6)) {
        printf("Disconnect int\n");
    }

    return status;
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

void USB0_IRQHandler(void)
{
    uint32_t length, i;
    uint32_t global_intr;
    uint32_t ep_intr;
    uint32_t epint;

    global_intr = USB_ReadConnectionInterrupt();
    ep_intr = USB_ReadDevAllEpInterrupt();
    /* ep 0 */
    if (ep_intr & BIT(0)) {
        epint = USB_ReadEPDetailedIntStatus(0);
        if (epint & BIT(20)) {
            length = REG32(USB_COM_CFG_BASE + 0x120); // Rx FIFO size
            printf("rd data from Ep0, len = %d\n", length);
            FIFORead(0, length, request.buffer);
            if (length != 0) {
                for (i = 0; i < length; i++) {
                    printf("%x ", request.buffer[i]);
                }
                printf("\n");

                if (length == 0x8) {
                    StatusEnd = 0;
                    SetupMaxLength = (request.buffer[6] | (unsigned short)request.buffer[7] << 8);
                    handle_usb_request(request);
                }
            } else {
                StatusEnd = 1;
            }
        }

        if (epint & BIT(26)) {  // ep0 DataNotReady
            if (setUpStage == 1) {
                UsbEp0Up();
            }
        }

        ep_intr &= ~BIT(0);
    }
    /* Ep 1..15 */
    uint32_t txis, rxis;
    txis = ep_intr & 0xfffe;
    while (txis) {
        unsigned const nEP = __builtin_ctz(txis);
        epint = USB_ReadEPDetailedIntStatus(nEP);
        if (epint & BIT(26)) {
            if (nEP == 1) {
                UsbEp1In();
            }
        }
        txis &= ~BIT(nEP);
    }

    rxis = (ep_intr & 0xfffe0000) >> 16;
    while (rxis) {
        unsigned const nEP = __builtin_ctz(rxis);
        epint = USB_ReadEPDetailedIntStatus(nEP);
        if (epint & BIT(20)) {
            if (nEP == 2) {
                UsbEp2Out();
            }
        }
        rxis &= ~BIT(nEP);
    }
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
        /* TxRxMaxPayload mode: bulk */
        if (g_speed_mode = USB_OTG_SPEED_FULL) {
            REG32(USB_COM_CFG_BASE + 0x118 + 0x40 * epnum) = 0x40;
        } else if (g_speed_mode = USB_OTG_SPEED_HIGH) {
            REG32(USB_COM_CFG_BASE + 0x118 + 0x40 * epnum) = 0x200;
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

void USB_DevInit(void)
{
    REG32(USB_COM_CFG_BASE + 0x14) &= ~BIT(4); // SOF int disable
    REG32(USB_COM_CFG_BASE + 0x1C) |= BIT(8);  // accelerate simulation of utmi control fsm, just for debug
    REG32(USB_COM_CFG_BASE + 0x24) = 0x8;      // The interval between send two commands

    USB_SetCurrentMode(USB_DEVICE_MODE);
    USB_SetDevSpeed(USB_OTG_SPEED_HIGH);
    // USB_SetDevSpeed(USB_OTG_SPEED_FULL);
    USB_SetDeviceConnectTime();

    /* epnum, in or out, fifo size */
    USB_ActivateEndpoint(0, USB_EP0, 0x40);
    USB_ActivateEndpoint(1, USB_EP_IN, 0x200 * 4);
    USB_ActivateEndpoint(2, USB_EP_OUT, 0x200 * 4);

    USB_PhyInit();

    REG32(USB_COM_CFG_BASE + 0xC0) &= ~BIT(0); // reset
    delay_1ms(2);
    REG32(USB_COM_CFG_BASE + 0xC0) |= BIT(0); // set
}

int main(void)
{
    printf("usb Upan test\r\n");

    __enable_irq();
    ECLIC_Register_IRQ(USB0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                                 ECLIC_LEVEL_TRIGGER, 1, 1,
                                 USB0_IRQHandler);

    USB_DevInit();

    while(1) {
        /* wait */
    }

    return 0;
}
