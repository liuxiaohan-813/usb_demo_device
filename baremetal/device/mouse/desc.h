#include "../../driver/usb.h"

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

const unsigned char DevDes[18]=
{
    0x12, /* bLength */
    0x01, /* bDescriptorType : device_descriptor */
    0x10,
    0x01, /* bcdUSB usb2.0 = 0x0200 usb1.1 = 0x0110 usb3.11 = 0x0311 */
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
    0x00, /* Index of serial number string */
    0x01  /* bNumConfigurations */
};

/* ReportDescriptor */
const unsigned char ReportDescriptor[]=
{
    0x05, 0x01, // Generic Desktop
    0x09, 0x02, // Mouse
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, // Usage (pointer)
    0xA1, 0x00, // Collection (physical)
    0x05, 0x09, // usage page (Button)
    0x19, 0x01, // usage Minimum (button 1)
    0x29, 0x03, // usage Maximum (button 3)
    0x15, 0x00, // logical Minimum (0)
    0x25, 0x01, // logical Maximum (1)
    0x75, 0x01, // report size (1)
    0x95, 0x03, // report count (3)
    0x81, 0x02, // input

    0x75, 0x05, // report size (5)
    0x95, 0x01, // report count (1)
    0x81, 0x03, // input

    0x05, 0x01, // Generic Desktop
    0x09, 0x30, // Usage (X)
    0x09, 0x31, // Usage (Y)
    0x09, 0x38, // Usage (wheel)
    0x15, 0x81, // logical Minimum (-127)
    0x25, 0x7F, // logical Maximum (127)
    0x75, 0x08, // report_size 8
    0x95, 0x03, // report count (3)
    0x81, 0x06, //input
    0xC0, // end collection
    0xC0  // end collection
};

/* dev Configurations */
const unsigned char ConDes[9 + 9 + 9 + 7]=
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
    0x01, // bNumEndpoints(not include Ep0)
    0x03, // bInterfaceClass (Hid)
    0x01, // bInterfaceSubClass
    0x02, // bInterfaceProtocol : mouse
    0x00, // iInterface
    // conf HID descriptor
    0x09, // bLength
    0x21, // bDescriptorType
    0x11,
    0x01, // bcdHID : 0x0111
    0x00, // bContryCode
    0x01, // bNumDescriptor
    0x22, // bDescriptorType
    sizeof(ReportDescriptor) & 0xFF,
    (sizeof(ReportDescriptor) >> 8) & 0xFF, // wDescriptorLength
    // Endpoint
    0x07, // bLength
    0x05, // bDescriptorType
    0x81, // Ep1 : In
    0x03, // bmAttributes : Interrupt
    0x04,
    0x00, // wMaxPackeSize : 0x0004
    0xA  // bInterval : 10ms
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
    0x12,
    0x03,
    0x68, 0x00, //hezaizai
    0x65, 0x00,
    0x7A, 0x00,
    0x61, 0x00,
    0x69, 0x00,
    0x7A, 0x00,
    0x61, 0x00,
    0x69, 0x00,
};
/* product Des */
const unsigned char Prod_Des[]=
{
    0x32,  // 修改长度：0x32 = 50字节 (25个Unicode字符 × 2 + 2字节头部)
    0x03,
    0x44, 0x00, // Demo USB2.0 optical Mouse
    0x65, 0x00,
    0x6D, 0x00,
    0x6F, 0x00,
    0x20, 0x00,
    0x55, 0x00,
    0x53, 0x00,
    0x42, 0x00,
    0x32, 0x00,
    0x2E, 0x00,
    0x30, 0x00,
    0x20, 0x00,
    0x6F, 0x00,
    0x70, 0x00,
    0x74, 0x00,
    0x69, 0x00,
    0x63, 0x00,
    0x61, 0x00,
    0x6C, 0x00,
    0x20, 0x00,
    0x4D, 0x00,
    0x6F, 0x00,
    0x75, 0x00,
    0x73, 0x00,
    0x65, 0x00,
};
/* product ser */
const unsigned char SerDes[18] =
{
    0x12,
    0x03,
    0x32, 0x00, // 20251024
    0x30, 0x00,
    0x32, 0x00,
    0x35, 0x00,
    0x31, 0x00,
    0x30, 0x00,
    0x32, 0x00,
    0x34, 0x00,
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