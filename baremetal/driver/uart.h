
#include "utils.h"

#ifndef _UART_H
#define _UART_H

#define USART0                              ((USART_TypeDef *)0x10080000)

#define USART_STATUS_TXIP                           BIT(0)                                          /*!< Transmit Watermark Interrupt Pending Flag*/
#define USART_STATUS_RXIP                           BIT(1)

#define USART_IE_TX                                 BIT(0)
#define USART_IE_RX                                 BIT(1)

/* ===== USART TXDATA Register definition ===== */
#define USART_TXDATA_FULL                           BIT(31)                                         /*!< USART TXDATA: Transmit FIFO Full Flag */

/* ===== USART RXDATA Register definition ===== */
#define USART_RXDATA_EMPTY                          BIT(31)

#define USART_RXCTRL_RXCNT(regval)                  BITS(16,19)

typedef struct {
    volatile uint32_t TXDATA;                  /*!< Offset: 0x000 (R/W)  USART TX Data Register */
    volatile uint32_t RXDATA;                  /*!< Offset: 0x004 (R/W)  USART RX Data Register */
    volatile uint32_t TXCTRL;                  /*!< Offset: 0x008 (R/W)  USART TX CTRL Register */
    volatile uint32_t RXCTRL;                  /*!< Offset: 0x00C (R/W)  USART RX CTRL Register */
    volatile uint32_t IE;                      /*!< Offset: 0x010 (R/W)  USART IE Register */
    volatile uint32_t STATUS;                 /*!< Offset: 0x014 (R/W)  USART Status Register */
    volatile uint32_t DIV;                     /*!< Offset: 0x018 (R/W)  USART Baudrate Divider Register */
    volatile uint32_t SETUP;                   /*!< Offset: 0x01C (R/W)  USART Setup Register */
    volatile uint32_t RX_IDLE_TOUT_NUM;        /*!< Offset: 0x044 (R/W)  USART RX IDLE Timeout Number Register*/
    volatile uint32_t RX_WM_TOUT_NUM;          /*!< Offset: 0x048 (R/W)  USART RX Watemark Timeout Number Register*/
    volatile uint32_t RX_FIFO_LEFT_ENTRY;      /*!< Offset: 0x04c (R/W)  USART RX FIFO Left Entry Register*/
    volatile uint32_t TX_FIFO_LEFT_ENTRY;      /*!< Offset: 0x050 (R/W)  USART TX FIFO Left Entry Register*/
    volatile uint32_t TX_DATA_SIZE;            /*!< Offset: 0x054 (R/W)  USART DMA TX Data Size Register*/
    volatile uint32_t RX_DATA_SIZE;            /*!< Offset: 0x058 (R/W)  USART DMA TX Data Size Register*/
    volatile uint32_t RESERVER1[9];
    volatile uint32_t IP_VERSION;              /*!< Offset: 0x080 (R/W)  USART Verison Register*/
} USART_TypeDef;

FlagStatus USART_GetITStatus(USART_TypeDef *USARTx, uint32_t USART_IT);
uint8_t USART_ReceiveData(USART_TypeDef *USARTx);
void USART_ITConfig(USART_TypeDef *USARTx, uint32_t USART_IE, ControlStatus Status);
void USART_Set_RxWaterMark(USART_TypeDef *USARTx, uint32_t Watermark);

#endif