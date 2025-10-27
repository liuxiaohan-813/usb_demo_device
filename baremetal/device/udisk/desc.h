#include "usb_sdk_hal.h"

const unsigned char DevDes[18]=
{
    0x12, /* bLength */
    0x01, /* bDescriptorType : device_descriptor */
    0x00,
    0x02, /* bcdUSB usb2.0 = 0x0200 usb1.1 = 0x0110 usb3.11 = 0x0311 */
    0x00, /* bDeviceClass */
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
    0x03, /* Index of serial number string */
    0x01  /* bNumConfigurations */
};

/* dev Configurations */
const unsigned char ConDes[9 + 9 + 7 + 7]=
{
    // Configuation Descriptor
    0x09, // bLength
    0x02, // bDescpriptorType
    sizeof(ConDes) & 0xFF,
    (sizeof(ConDes)) >> 8 & 0xFF, // wTotalLength
    0x01, // bNumInterfaces
    0x01, // bConfigurationValue
    0x00, // iConfiguration
    0x80, // bmAttributes
    0x32, // bMaxPower : 100mA

    // Interface
    0x09, // bLength
    0x04, // bDescriptorType
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndpoints(not include Ep0)
    0x08, // bInterfaceClass (Mass Storage)
    0x06, // bInterfaceSubClass
    0x50, // bInterfaceProtocol : bulk-only
    0x00, // iInterface

    // Endpoint
    0x07, // bLength
    0x05, // bDescriptorType
    0x81, // Ep1 : In
    0x02, // bmAttributes : bulk
    0x00,
    0x02, // wMaxPackeSize : 0x0200
    0x00, // bInterval
    
    0x07,
    0x05,
    0x02, // Ep2 : Out
    0x02, // bmAttributes : bulk
    0x00,
    0x02, // wMaxPackeSize : 0x0200
    0x00
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
    34,
    0x03,
    0x4e, 0x00, // Nuclei USB2.0 Upan
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
    0x55, 0x00,
    0xd8, 0x76,
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

/* Upan class */
const uint8_t MaxLun[1] = {0}; // max is 15