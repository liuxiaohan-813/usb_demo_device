#include "uart.h"

FlagStatus USART_GetITStatus(USART_TypeDef *USARTx, uint32_t USART_IT)
{
    if (USARTx->STATUS & USART_IT) {
        return SET;
    } else {
        return RESET;
    }
}

uint8_t USART_ReceiveData(USART_TypeDef *USARTx)
{
    uint32_t reg;

    do {
        reg = USARTx->RXDATA;
    }
    while (reg & USART_RXDATA_EMPTY);
    return (uint8_t)(reg & 0xFF);
}

void USART_ITConfig(USART_TypeDef *USARTx, uint32_t USART_IE, ControlStatus Status)
{
    if (Status) {
        USARTx->IE |= (USART_IE);
    } else {
        USARTx->IE &= ~(USART_IE);
    }
}

void USART_Set_RxWaterMark(USART_TypeDef *USARTx, uint32_t Watermark)
{
    USARTx->RXCTRL |= USART_RXCTRL_RXCNT(Watermark);
}