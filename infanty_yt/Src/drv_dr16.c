#include "drv_dr16.h"
#include "stm32f4xx_hal_uart.h"
#include "main.h"
#include <string.h>

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);

uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];

void dr16_uart_init(UART_HandleTypeDef *huart)
{
  UART_Receive_DMA_No_IT(huart, dr16_uart_rx_buff, DR16_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

uint8_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart)
{
  uint8_t rflag;
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle dbus data dbus_buf from DMA */
    if ((DR16_RX_BUFFER_SIZE - huart->hdmarx->Instance->NDTR) != DR16_DATA_LEN)
    {
      memset(dr16_uart_rx_buff, 0, DR16_RX_BUFFER_SIZE);
      rflag = 1;
    }
    else
      rflag = 0;
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DR16_RX_BUFFER_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
    return rflag;
  }
  else
  {
    return 1;
  }
}

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
  uint32_t tmp = 0;

  tmp = huart->RxState;
  if (tmp == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t) &huart->Instance->DR,
      (uint32_t) pData, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
     in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
