#include "can.h"

CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef * hcan)
{
  GPIO_InitTypeDef gpio = {0};
  if (hcan->Instance != CAN1) return;
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &gpio);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef * hcan)
{
  if (hcan->Instance != CAN1) return;
  __HAL_RCC_CAN1_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
}
