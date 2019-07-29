/*
 * neo6m.c
 *
 *  Created on: Apr 18, 2019
 *      Author: raistlin
 */

#include <stdlib.h>
#include <string.h>

#include "commons/commons.h"
#include "gps/neo6m.h"
#include "stm32vldiscovery_utils.h"
#include "time/kov_time.h"
#include "nmea.h"

#define NEO6_PORT     GPIOB
#define NEO6_PIN_TX   GPIO_Pin_10
#define NEO6_PIN_RX   GPIO_Pin_11
#define NEO6_PIN_VCC  GPIO_Pin_12

volatile bool neo6m_needParseCurrentBuff = false;

//if something goes wrong - make volatile
static char internalBuffer[NMEA_MAX_LENGTH+1] = {0};
static uint8_t bufferPos = 0;
///////////////////////////////////////////////////////

void neo6m_power(bool on) {
  void (*gpio_change)(GPIO_TypeDef*, uint16_t) = on ? GPIO_SetBits : GPIO_ResetBits;
  gpio_change(NEO6_PORT, NEO6_PIN_VCC);
}
///////////////////////////////////////////////////////

void neo6m_init() {
  /* USART configuration structure for USART3 */
  USART_InitTypeDef usart3_cfg;
  /* Bit configuration structure for GPIOA PIN9 and PIN10 */
  GPIO_InitTypeDef gpiob_cfg;
  NVIC_InitTypeDef nvicInitCfg;

  /* Enalbe clock for USART3, AFIO and GPIOB */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  gpiob_cfg.GPIO_Pin = NEO6_PIN_TX;
  gpiob_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpiob_cfg.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(NEO6_PORT, &gpiob_cfg);

  gpiob_cfg.GPIO_Pin = NEO6_PIN_RX;
  gpiob_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpiob_cfg.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(NEO6_PORT, &gpiob_cfg);

  gpiob_cfg.GPIO_Pin = NEO6_PIN_VCC;
  gpiob_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpiob_cfg.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NEO6_PORT, &gpiob_cfg);

  /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do Rx, No HW flow control
     */
  usart3_cfg.USART_BaudRate = 9600;
  usart3_cfg.USART_WordLength = USART_WordLength_8b;
  usart3_cfg.USART_StopBits = USART_StopBits_1;
  usart3_cfg.USART_Parity = USART_Parity_No;
  usart3_cfg.USART_Mode = USART_Mode_Rx;
  usart3_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* Configure USART3 */
  USART_Init(USART3, &usart3_cfg);
  /* Enable RXNE interrupt */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  /* Enable USART3 global interrupt */
  /* Enable the USART3 event Interrupt */
  nvicInitCfg.NVIC_IRQChannel = USART3_IRQn;
  nvicInitCfg.NVIC_IRQChannelPreemptionPriority = 1;
  nvicInitCfg.NVIC_IRQChannelSubPriority = 0;
  nvicInitCfg.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&nvicInitCfg);

  /* Enable USART3 */
  USART_Cmd(USART3, ENABLE);
}
///////////////////////////////////////////////////////

static void put_char(char c) {
  internalBuffer[bufferPos] = c;
  internalBuffer[++bufferPos] = 0;
}

static volatile uint8_t cs = 0;
static void rx_start(char c) {
  if (c != '$' && c != '!')
    return;
  cs = 1;
  put_char(c);
}

static inline void rx_put(char c) {
  put_char(c);
  if (c == '\n' && internalBuffer[bufferPos-2] == '\r') {
    NVIC_DisableIRQ(USART3_IRQn);
    neo6m_needParseCurrentBuff = true;
    cs = 0;
    bufferPos = 0;
  }

  if (bufferPos > NMEA_MAX_LENGTH) {
    cs = 0;
    bufferPos = 0;
  }
}
///////////////////////////////////////////////////////

void USART3_IRQHandler() {
  static void (*dfa[2])(char) = { rx_start, rx_put };
  uint16_t sr = USART3->SR;
  char c = (char)USART_ReceiveData(USART3);
  if (!(sr & USART_SR_RXNE))
    return;
  dfa[cs](c);
}
///////////////////////////////////////////////////////

static void convert_rmc2neo6(const struct nmea_sentence_rmc *rmc, neo6m_data_t *neo6) {
  neo6->date = rmc->date;
  if (rmc->date.year < 1900)
    neo6->date.year += 2000; //HACK!
  neo6->time = rmc->time;

  neo6->speed_knots = nmea_tofloat(&rmc->speed);
  neo6->magnetic_variation = nmea_tofloat(&rmc->variation);
  neo6->course = nmea_tofloat(&rmc->course);

  neo6->longitude = nmea_tocoord(&rmc->longitude);
  neo6->latitude = nmea_tocoord(&rmc->latitude);
}
///////////////////////////////////////////////////////

neo6m_data_t neo6m_parseCurrentBuff(bool *valid) {
  neo6m_data_t res;
  *valid = false;
  memset(&res, 0, sizeof(neo6m_data_t));
  enum nmea_sentence_id sid = nmea_sentence_id(internalBuffer, false);
  struct nmea_sentence_rmc rmc;
  switch(sid) {
    case NMEA_SENTENCE_RMC:
      if ((*valid = nmea_parse_rmc(&rmc, internalBuffer))) {
        if ((*valid = rmc.valid))
          convert_rmc2neo6(&rmc, &res);
      }
      break;
    default:
      break;
  }  
  NVIC_EnableIRQ(USART3_IRQn);
  return res;
}
///////////////////////////////////////////////////////

void neo6m_interrupts(bool enable) {
  if (enable) NVIC_EnableIRQ(USART3_IRQn);
  else NVIC_DisableIRQ(USART3_IRQn);
}
