#include "usb_sdk_hal.h"

const unsigned char DevDes[18]=
{
    0x12, /* bLength */
    0x01, /* bDescriptorType : device_descriptor */
    0x10,
    0x01, /* bcdUSB usb2.0 = 0x0200 usb1.1 = 0x0110 usb3.11 = 0x0311 */
    0x02, /* bDeviceClass (Notice: should be 0x02)*/
    0x00, /* bDeviceSubClass */
    0x00, /* bDeviceProtocol */
    0x40, /* bMaxPacketSize */
    0x66,
    0x66, /* idVendor : 2 Bytes */
    0x88,
    0x88, /* idProduct : 2 Bytes */
    0x01,
    0x00, /* bcdDevice rel. 1.00 */
    0x01, /* Index of manufacturer string */
    0x02, /* Index of product string */
    0x00, /* Index of serial number string */
    0x01  /* bNumConfigurations */
};

/* dev Configurations */
const unsigned char ConDes[9 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7]=
{
    // Configuation Descriptor
    0x09, // bLength
    0x02, // bDescpriptorType
    sizeof(ConDes) & 0xFF,
    (sizeof(ConDes)) >> 8 & 0xFF, // wTotalLength
    0x02, // bNumInterfaces
    0x01, // bConfigurationValue
    0x00, // iConfiguration
    0x80, // bmAttributes
    0x32, // bMaxPower : 100mA

    // Interface 0 (CDC)
    0x09, // bLength
    0x04, // bDescriptorType
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints(exclude Ep0)
    0x02, // bInterfaceClass (CDC)
    0x02, // bInterfaceSubClass (Abstract Control Model)
    0x01, // bInterfaceProtocol : (Common AT Commands)
    0x00, // iInterface

    // functional descriptors
    0x05, // bLength: Endpoint Descriptor size
    0x24, // bDescriptorType: CS_INTERFACE
    0x00, // bDescriptorSubtype: Call Management Func Desc
    0x10,
    0x01, // bcdCDC: spec release number : 0x0110

	0x05, // bFunctionLength
    0x24, // bDescriptorType
    0x01, // bDescriptorSubtype: Call Management Func Desc
    0x00, // bmCapabilities: D0 + D1
    0x00, // bDataInterface: 1

	0x04, // bFunctionLength
    0x24, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: Abstract Control Management desc
    0x02, // bmCapabilities suport Set_Line_Coding Set_Control_Line_State Get_Line_Coding Serial_State

    0x05, // bFunctionLength
    0x24, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptorSubtype: Union func desc
    0x00, // bMasterInterface: Communication class interface
    0x01, // bSlaveInterface0: Data Class Interface
    // Endpoint 1
    0x07, // bLength
    0x05, // bDescriptorType
    0x81, // Ep1 : In
    0x03, // bmAttributes : Interrupt
    0x40,
    0x00, // wMaxPackeSize : 0x0040
    0xFF,  // bInterval : 256ms

    // Interface 1 (Data)
    0x09, // bLength
    0x04, // bDescriptorType
    0x01, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndpoints(exclude Ep0)
    0x0A, // bInterfaceClass (CDC-Data)
    0x00, // bInterfaceSubClass (Abstract Control Model)
    0x00, // bInterfaceProtocol : (Common AT Commands)
    0x00, // iInterface

    // Endpoint 2
    0x07, // bLength
    0x05, // bDescriptorType
    0x82, // Ep2 : In
    0x02, // bmAttributes : Bulk
    0x40,
    0x00, // wMaxPackeSize : 0x0040
    0xF,  // bInterval
    // Endpoint 3
    0x07, // bLength
    0x05, // bDescriptorType
    0x3,  // Ep3 : Out
    0x02, // bmAttributes : Bulk
    0x40,
    0x00, // wMaxPackeSize : 0x0040
    0x00  // bInterval
};

/* langDes */
const unsigned char LangDes[]=
{
    0x04,
    0x03,
    0x09,
    0x04
};
/* Manuf */
const unsigned char Manuf_Des[]=
{
    0xE,
    0x03,
    0x4e, 0x00, // Nuclei
    0x75, 0x00,
    0x63, 0x00,
    0x6c, 0x00,
    0x65, 0x00,
    0x69, 0x00,
};
/* product Des */
const unsigned char Prod_Des[]=
{
    36,
    0x03,
    0x4e, 0x00, // Nuclei USB2.0 CDC
    0x75, 0x00,
    0x63, 0x00,
    0x6c, 0x00,
    0x65, 0x00,
    0x69, 0x00,
    0x20, 0x00,
    0x55, 0x00,
    0x53, 0x00,
    0x42, 0x00,
    0x32, 0x00,
    0x2e, 0x00,
    0x30, 0x00,
    0x20, 0x00,
    0x43, 0x00,
    0x44, 0x00,
    0x43, 0x00,
};
/* product ser */
const unsigned char SerDes[18] =
{
    0x12,
    0x03,
    0x32, 0x00, // 20220522
    0x30, 0x00,
    0x32, 0x00,
    0x32, 0x00,
    0x30, 0x00,
    0x35, 0x00,
    0x32, 0x00,
    0x32, 0x00,
};

uint8_t DeviceQualifierDesc[] =
{
    0x0A,
    0x06,
    0x10,
    0x01,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

// cdc para 57600, 1, None, 8bit
unsigned char LineCoding[7] =
{
    0x80,
    0x25,
    0x00,
    0x00,
    0x00,
    0x00,
    0x08
};