#include <stdio.h>
#include "../../driver/uart.h"
#include "desc.h"

unsigned char mVarSetupRequest;
uint16_t mVarSetupLength;
const unsigned char *VarSetupDescr = NULL;
uint16_t SetupMaxLength = 0;
uint8_t setUpStage = 0;
uint8_t StatusEnd = 0;
uint8_t EP0_MAX_LEN = DevDes[7];

#define DMA_WRITE  0
#define DMA_READ   1

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

mREQUEST_PACKET request;

void handle_get_desc_request(mREQUEST_PACKET USB_request)
{
    int len;
    unsigned char req_type = USB_request.r.wValue >> 8;
    unsigned char req_index = USB_request.r.wValue & 0xF;

    switch (req_type) {
        case 1:
            printf("device desc id(%d)\n",req_index);
            VarSetupDescr = DevDes;
            mVarSetupLength = MIN(sizeof(DevDes), SetupMaxLength);
            break;
        case 2:
            printf("cfg desc id(%d)\n",req_index);
            VarSetupDescr = ConDes;
            mVarSetupLength = MIN(sizeof(ConDes), SetupMaxLength);
            break;
        case 3:
            printf("string desc：");
            if ((USB_request.r.wValue & 0xff) == 0) {
                printf("LangDes\n");
                VarSetupDescr = LangDes;
                mVarSetupLength = MIN(sizeof(LangDes), SetupMaxLength);
            } else if ((USB_request.r.wValue & 0xff) == 1) {
                printf("Manuf_Des id(%d)\n",req_index);
                VarSetupDescr = Manuf_Des;
                mVarSetupLength = MIN(sizeof(Manuf_Des), SetupMaxLength);
            } else if ((USB_request.r.wValue & 0xff) == 2) {
                printf("Prod_Des id(%d)\n",req_index);
                VarSetupDescr = Prod_Des;
                mVarSetupLength = MIN(sizeof(Prod_Des), SetupMaxLength);
            } else if ((USB_request.r.wValue & 0xff) == 3) {
                printf("SerDes id(%d)\n",req_index);
                VarSetupDescr = SerDes;
                mVarSetupLength = MIN(sizeof(SerDes), SetupMaxLength);
            } else {
                printf("other\n");
            }
            break;
        case 4:
            printf("interface desc id(%d)\n",req_index);
            break;
        case 5:
            printf("endpoint desc id(%d)\n",req_index);
            break;
        case 6:
            printf("qualifier desc id(%d)\n",req_index);
            VarSetupDescr = DeviceQualifierDesc;
            mVarSetupLength = MIN(sizeof(DeviceQualifierDesc), SetupMaxLength);
            break;
        case 0x22:
            printf("hid desc\n");
            VarSetupDescr = ReportDescriptor;
            mVarSetupLength = MIN(sizeof(ReportDescriptor), SetupMaxLength);
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

enum bRequest_ID {
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    RESERVED0 = 2,
    SET_FEATURE = 3,
    RESERVED1 = 4,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12,
};

// only handle GET_DESCRIPTOR and SET_ADDRESS req
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
            printf("SET_ADDRESS:\r\n");
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
            handle_std_request(USB_request); //handle standard request
            break;
        case 1:
            printf("class request->");
            break;
        case 2:
            printf("manufacture request->");
            break;
        case 3:
            printf("reserve req!\r\n");
            break;
    }
}

unsigned char buf_mouse[4];
unsigned char mouse_data_ready;
unsigned char need_release = 0;

void SendMouseReport(void)
{
    FIFOWrite(1, 4, buf_mouse);
}

void USART0_IRQHandler(void)
{
    uint8_t USART0_receive_ch;

    if (USART_GetITStatus(USART0, USART_STATUS_RXIP) == SET) {
        USART0_receive_ch = USART_ReceiveData(USART0);
        
        // 鼠标移动指令
        if (USART0_receive_ch == 'a') {
            buf_mouse[1] = -10;  // X位移：左移
            buf_mouse[2] = 0;    // Y位移：不变
            printf("left\n");
            need_release = 0;   // 移动事件不需要释放，所以清除need_release（防止点击事件未完成）
        } else if (USART0_receive_ch == 'd') {
            buf_mouse[1] = 10;   // X位移：右移
            buf_mouse[2] = 0;    // Y位移：不变
            printf("right\n");
            need_release = 0;
        } else if (USART0_receive_ch == 'w') {
            buf_mouse[1] = 0;    // X位移：不变
            buf_mouse[2] = -10;  // Y位移：上移
            printf("up\n");
            need_release = 0;
        } else if (USART0_receive_ch == 's') {
            buf_mouse[1] = 0;    // X位移：不变
            buf_mouse[2] = 10;   // Y位移：下移
            printf("down\n");
            need_release = 0;
        }
        // 鼠标按键指令
        else if (USART0_receive_ch == 'c') {
            // 左键点击：设置按钮状态bit0为1
            buf_mouse[0] = 0x01;  // 按钮状态：左键按下
            buf_mouse[1] = 0;     // X位移：不变
            buf_mouse[2] = 0;     // Y位移：不变
            printf("left click\n");
            need_release = 1;    // 标记需要发送释放报告
        } else if (USART0_receive_ch == ' ') {
            // 右键点击：设置按钮状态bit1为1
            buf_mouse[0] = 0x02;  // 按钮状态：右键按下
            buf_mouse[1] = 0;     // X位移：不变
            buf_mouse[2] = 0;     // Y位移：不变
            printf("right click\n");
            need_release = 1;
        } else if (USART0_receive_ch == 'x') {
            // 中键点击：设置按钮状态bit2为1
            buf_mouse[0] = 0x04;  // 按钮状态：中键按下
            buf_mouse[1] = 0;     // X位移：不变
            buf_mouse[2] = 0;     // Y位移：不变
            printf("middle click\n");
            need_release = 1;
        } else if (USART0_receive_ch == 'r') {
            // 释放所有按键
            buf_mouse[0] = 0x00;  // 按钮状态：所有键释放
            buf_mouse[1] = 0;     // X位移：不变
            buf_mouse[2] = 0;     // Y位移：不变
            printf("release all buttons\n");
            need_release = 0;     // 不需要再释放
        }

        mouse_data_ready = 1;
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
    uint32_t ep_int_node;
    uint32_t ep_int_status;

    global_intr = USB_ReadConnectionInterrupt();
    ep_int_node = USB_ReadDevAllEpInterrupt();
    /* ep 0 */
    if (ep_int_node & BIT(0)) {
        ep_int_status = USB_ReadEPDetailedIntStatus(0);
        if (ep_int_status & BIT(20)) { //ep0 rx package ready
            length = REG32(USB_COM_CFG_BASE + 0x120); //get Rx FIFO package size
            FIFORead(0, length, request.buffer);
            if (length != 0) {
                printf("\nrd data from Ep0, len = %d\n", length);
                for (i = 0; i < length; i++) {
                    printf("0x%02x ", request.buffer[i]);
                }
                printf("\n");

                if (length == 0x8) {
                    StatusEnd = 0;
                    SetupMaxLength = (request.r.wLength);
                    handle_usb_request(request); // handle request package
                }
            } else {
                printf("receive ACK\r\n");
                StatusEnd = 1;
            }
        }

        if (ep_int_status & BIT(26)) {  // wait for ep0 tx Data Ready
            if (setUpStage == 1) {
                UsbEp0Up();
            }
        }

        ep_int_node &= ~BIT(0);
    }
    /* Ep 1..15 */
    uint32_t txis, rxis;
    txis = ep_int_node & 0xfffe;
    while (txis) {
        unsigned const nEP = __builtin_ctz(txis);
        ep_int_status = USB_ReadEPDetailedIntStatus(nEP);
        if (ep_int_status & BIT(26)) {
            if (nEP == 1) { // use ep1 to return value
                if (mouse_data_ready) {
                    // 发送当前的鼠标报告
                    SendMouseReport();
                    if (need_release) {
                        // 准备释放报告
                        buf_mouse[0] = 0;  // 释放所有按键
                        need_release = 0;   // 清除标志
                        mouse_data_ready = 1; // 再次设置标志，发送释放报告
                    } else {
                        mouse_data_ready = 0; // 清除标志
                    }
                }
            }
        }
        txis &= ~BIT(nEP);
    }

    rxis = (ep_int_node & 0xfffe0000) >> 16;
    while (rxis) {
        unsigned const nEP = __builtin_ctz(rxis);
        ep_int_status = USB_ReadEPDetailedIntStatus(nEP);
        if (ep_int_status & BIT(20)) {

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

void USB_DevInit(void)
{
    REG32(USB_COM_CFG_BASE + 0x14) &= ~BIT(4); // disable SOF int
    // REG32(USB_COM_CFG_BASE + 0x1C) |= BIT(8);  // accelerate simulation of utmi control fsm, just for debug

    /* device mode and full speed mode */
    USB_SetCurrentMode(USB_DEVICE_MODE);
    USB_SetDevSpeed(USB_OTG_SPEED_FULL);
    USB_SetDeviceConnectTime();

    /* epnum, in or out, fifo size */
    USB_ActivateEndpoint(0, USB_EP0, 0x40);
    USB_ActivateEndpoint(1, USB_EP_IN, 0x40);

    USB_PhyInit();

    /* reset phy */
    REG32(USB_COM_CFG_BASE + 0xC0) &= ~BIT(0); // reset
    delay_1ms(2);
    REG32(USB_COM_CFG_BASE + 0xC0) |= BIT(0); // set
}

/*
  * The movement of the mouse cursor is simulated by using uart to
  * recognize the command of moving characters.Then it is returned
  * to the host through the usb interface.
  * command of moving characters：a->left、s->down、w->up、d->right
*/

int main(void)
{
    printf("usb mouse device test\r\n");

    /* enable global initerrupt */
    __enable_irq();

    /* uart0 config */
    ECLIC_Register_IRQ(USART0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                                    ECLIC_LEVEL_TRIGGER, 2, 1,
                                    USART0_IRQHandler);
    USART_Set_RxWaterMark(USART0, 0);
    USART_ITConfig(USART0, USART_IE_RX, ENABLE);

    /* usb config */
    ECLIC_Register_IRQ(USB0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                                 ECLIC_LEVEL_TRIGGER, 1, 1,
                                 USB0_IRQHandler);
    USB_DevInit();

    while(1) {}

    return 0;
}
