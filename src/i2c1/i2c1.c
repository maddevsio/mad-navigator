#include <stddef.h>
#include "i2c1/i2c1.h"
#include "commons/commons.h"
#include "display/display.h"

#define I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define I2C_Port             GPIOB
#define I2C_SCL_Pin          GPIO_Pin_6
#define I2C_SDA_Pin          GPIO_Pin_7
#define I2C_Speed            200000

typedef enum {
  I2CEV_StartBitSent = 0, //SB
  I2CEV_AddresSent, //ADDR
  I2CEV_DataByteTransferFinished, //BTF
  I2CEV_ReceiveBufferNotEmpty, //RxNE
  I2CEV_TransmitBufferEmpty //TxE
} I2CEvent;

typedef enum {
  I2CER_ErrClass = 5,
  I2CER_BusError, //BERR
  I2CER_ArbitrationLoss, //ARLO
  I2CER_AckFail, //NACK
  I2CER_Overrun, //OVR
  I2CER_PEC, //PECERR
  I2CER_Timeout, //TIMEOUT
  I2CER_SMBBusAlert //SMBBUSALERT
} I2CError;
///////////////////////////////////////////////////////

//read multiple bytes context
typedef struct RMBContext {
  I2C_TypeDef *i2c;
  volatile uint8_t *dstBuff;
  volatile int8_t *pFinishCode;
  i2c1_pf_callback cb;
  volatile uint8_t readLen;
  uint8_t slaveAddr;
  uint8_t dstReg;
  uint8_t padding[1];
} RMBContext_t;

//write multiple bytes context
typedef struct WMBContext {
  I2C_TypeDef *i2c;
  volatile uint8_t *srcBuff;
  volatile int8_t *pFinishCode;
  i2c1_pf_callback cb;
  uint8_t slaveAddr;
  uint8_t dstReg;
  volatile uint8_t writeLen;
  uint8_t padding[1];
} WMBContext_t;
///////////////////////////////////////////////////////

#define I2C_ERR_MASK (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT | I2C_SR1_AF)
#define DFA_INIT_STATE 0
#define POSSIBLE_DFA_SIGNALS_N 6
#define I2C1_LONGEST_DFA 6

struct I2CAsynDFA;
typedef struct I2CAsyncState {
  int32_t ix;
  void *arg; //action arg
  void (*actions[POSSIBLE_DFA_SIGNALS_N]) (struct I2CAsyncState*, struct I2CAsynDFA *dfa);
} I2CAsyncState_t;
///////////////////////////////////////////////////////

typedef struct I2CAsynDFA {
  volatile int32_t cs; //current state
  I2CAsyncState_t states[I2C1_LONGEST_DFA];
} I2CAsynDFA_t;
///////////////////////////////////////////////////////

static I2CAsynDFA_t m_dfaI2C1;
static I2CAsynDFA_t preInitDFA(void);
static uint8_t DFASignal(I2C_TypeDef *i2c);
static void DFANext(I2C_TypeDef *i2c, I2CAsynDFA_t *dfa);
static void dfaNop(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void i2c1_reset(void);

I2CAsynDFA_t preInitDFA() {
  I2CAsynDFA_t res;
  static I2CAsyncState_t states[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {dfaNop, dfaNop, dfaNop, dfaNop, dfaNop, dfaNop}}, //init
  };
  res.cs = DFA_INIT_STATE;

  for (size_t i = 0; i < sizeof(states)/sizeof(I2CAsyncState_t); ++i) {
    states[i].ix = (int32_t)i;
    res.states[i] = states[i];
  }
  return res;
}
///////////////////////////////////////////////////////

uint8_t DFASignal(I2C_TypeDef *i2c) {
  volatile uint16_t sr1, sr2 = 0;
  sr1 = i2c->SR1;

  if (sr1 & I2C_SR1_SB)
    return I2CEV_StartBitSent;

  if (sr1 & I2C_SR1_ADDR) {
    sr2 = i2c->SR2; //clear addr register if set
    return I2CEV_AddresSent;
  }

  if (sr1 & I2C_SR1_BTF)
    return I2CEV_DataByteTransferFinished;

  if (sr1 & I2C_SR1_RXNE)
    return I2CEV_ReceiveBufferNotEmpty;

  if (sr1 & I2C_SR1_TXE)
    return I2CEV_TransmitBufferEmpty;

  if (sr1 & I2C_ERR_MASK)
    return I2CER_ErrClass;
  return 0xff;
}
///////////////////////////////////////////////////////

void DFANext(I2C_TypeDef *i2c, I2CAsynDFA_t *dfa) {
  uint8_t sig;
  int32_t cs;
  sig = DFASignal(i2c);
  if (sig != 0xff) {
    cs = dfa->cs;
    dfa->states[cs].actions[sig](&dfa->states[cs], dfa);
  }
}
///////////////////////////////////////////////////////

void I2C1_EV_IRQHandler() {
  DFANext(I2C1, &m_dfaI2C1);
}
///////////////////////////////////////////////////////

void I2C1_ER_IRQHandler() {
  DFANext(I2C1, &m_dfaI2C1);
}
///////////////////////////////////////////////////////

void i2c1_reset(void) {  
  I2C1->CR1 |= I2C_CR1_SWRST;
  //todo reset devices
}
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

static void rmbI2CErr(struct I2CAsyncState* st, struct I2CAsynDFA *dfa);
static void rmbDFAErr(struct I2CAsyncState* st, struct I2CAsynDFA *dfa);
static void rmbGenStart(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void rmbSendAddrTransmitter(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void rmbSendStartReg(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void rmbSendAddrReceiver(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void rmbReadByteBTF(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void rmbFinish(struct I2CAsyncState* st, struct I2CAsynDFA *dfa);

static void rmbI2CErr_Berr(RMBContext_t *ctx);
static void rmbI2CErr_AF(RMBContext_t *ctx);
static void rmbI2CErr_Arlo(RMBContext_t *ctx);
static void rmbI2CErr_Ovr(RMBContext_t *ctx);
static void rmbI2CErr_Timeout(RMBContext_t *ctx);

static I2CAsynDFA_t rmbDFA(uint8_t slaveAddr, uint8_t startReg,
                           volatile uint8_t *dstBuff,
                           uint8_t readLen,
                           volatile int8_t *pErr,
                           i2c1_pf_callback cb);

void rmbI2CErr_Berr(RMBContext_t *ctx) {
  if (ctx->i2c->SR1 & I2C_IT_SB)
    i2c1_reset(); //if berr set and sb set we need to restart i2c at all.
}
///////////////////////////////////////////////////////

void rmbI2CErr_AF(RMBContext_t *ctx) {  
  I2C_GenerateSTOP(ctx->i2c, ENABLE); //clear busy flag
}
///////////////////////////////////////////////////////

void rmbI2CErr_Arlo(RMBContext_t *ctx) {
  UNUSED(ctx);
}
///////////////////////////////////////////////////////

void rmbI2CErr_Ovr(RMBContext_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void rmbI2CErr_Timeout(RMBContext_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void rmbI2CErr(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  volatile uint16_t sr1;
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  sr1 = pctx->i2c->SR1;
  pctx->i2c->SR1 = (sr1 & ~I2C_ERR_MASK);

  if (sr1 & I2C_SR1_BERR)
    rmbI2CErr_Berr(pctx);
  if (sr1 & I2C_SR1_AF)
    rmbI2CErr_AF(pctx);
  if (sr1 & I2C_SR1_ARLO)
    rmbI2CErr_Arlo(pctx);
  if (sr1 & I2C_SR1_OVR)
    rmbI2CErr_Ovr(pctx);
  if (sr1 & I2C_SR1_TIMEOUT)
    rmbI2CErr_Timeout(pctx);

  I2C_AcknowledgeConfig(pctx->i2c, ENABLE);
  if (pctx->cb != NULL)
    pctx->cb(FC_I2C_ERR);
  dfa->cs = DFA_INIT_STATE;
  *pctx->pFinishCode = FC_I2C_ERR;
}
///////////////////////////////////////////////////////

//todo: we can't be here. SO! just restart device and i2c peripheral
void rmbDFAErr(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);  
  I2C_AcknowledgeConfig(pctx->i2c, ENABLE);
  I2C_GenerateSTOP(pctx->i2c, ENABLE);

  i2c1_reset();
  if (pctx->cb != NULL)
    pctx->cb(FC_DFA_ERR);

  dfa->cs = DFA_INIT_STATE;
  *pctx->pFinishCode = FC_DFA_ERR;
}
///////////////////////////////////////////////////////

void rmbGenStart(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  while (I2C_GetFlagStatus(pctx->i2c, I2C_FLAG_BUSY))
    __NOP();
  dfa->cs = 1;
  I2C_GenerateSTART(pctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void rmbSendAddrTransmitter(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  dfa->cs = 2;
  I2C_Send7bitAddress(pctx->i2c, pctx->slaveAddr, I2C_Direction_Transmitter);
}
///////////////////////////////////////////////////////

void rmbSendStartReg(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  dfa->cs = 3;
  I2C_SendData(pctx->i2c, pctx->dstReg);
  I2C_GenerateSTART(pctx->i2c, ENABLE); //re-start after this byte transmitted.
}
///////////////////////////////////////////////////////

void dfaNop(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  UNUSED(st);
  UNUSED(dfa);
}
///////////////////////////////////////////////////////

void rmbSendAddrReceiver(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  I2C_Send7bitAddress(pctx->i2c, pctx->slaveAddr, I2C_Direction_Receiver);
  dfa->cs = 4;
}
///////////////////////////////////////////////////////

void rmbReadByteBTF(struct I2CAsyncState *st,
                    struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  if (pctx->readLen > 3) {
    *pctx->dstBuff++ = (uint8_t)pctx->i2c->DR;
    --pctx->readLen;
    return;
  }

  I2C_AcknowledgeConfig(pctx->i2c, DISABLE); //Send NACK after last byte received
  *pctx->dstBuff++ = (uint8_t)pctx->i2c->DR; //read N-2
  I2C_GenerateSTOP(pctx->i2c, ENABLE); //generate stop after last byte received
  *pctx->dstBuff++ = (uint8_t)pctx->i2c->DR; //read N-1
  pctx->readLen -= 2;
  dfa->cs = 5;
}
///////////////////////////////////////////////////////

void rmbFinish(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);

  if (pctx->readLen == 1)
    *pctx->dstBuff++ = (uint8_t)pctx->i2c->DR; //read N-1 and clear rxne flag

  if (pctx->cb != NULL)
    pctx->cb(FC_FINISHED);

  I2C_AcknowledgeConfig(pctx->i2c, ENABLE);
  dfa->cs = DFA_INIT_STATE; //goto init state
  *pctx->pFinishCode = FC_FINISHED;
}
///////////////////////////////////////////////////////

static void rmbAddrReceivedLen1(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  UNUSED(dfa);
  RMBContext_t *pctx = (RMBContext_t*)(st->arg);
  I2C_AcknowledgeConfig(pctx->i2c, DISABLE); //Send NACK after last byte received
  I2C_GenerateSTOP(pctx->i2c, ENABLE); //generate stop after last byte received
}
///////////////////////////////////////////////////////

I2CAsynDFA_t rmbDFA(uint8_t slaveAddr,
                    uint8_t startReg,
                    volatile uint8_t *dstBuff,
                    uint8_t readLen,
                    volatile int8_t *pErr,
                    i2c1_pf_callback cb) {
  I2CAsynDFA_t res;
  static RMBContext_t rmbCtx;

  // SB, ADDR, BTF, RxNE, TxE, ERR
  static I2CAsyncState_t statesGE2[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbI2CErr}}, //init. if some interrupt happens here - it's NOT OK!
    //1
    {.actions = {rmbSendAddrTransmitter, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbI2CErr}}, //start sent. send i2c slave addr (transmitter mode)
    //2
    {.actions = {rmbDFAErr, rmbSendStartReg, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbI2CErr}}, //addr sent. send read register addr. TxE not set in addr stage
    //3
    {.actions = {rmbSendAddrReceiver, rmbDFAErr, dfaNop, rmbDFAErr, dfaNop, rmbI2CErr}}, //start sent. send i2c slave addr (receiver mode)
    //4
    {.actions = {rmbDFAErr, dfaNop, rmbReadByteBTF, dfaNop, dfaNop, rmbI2CErr}}, //addr sent. wait for byte received
    //5
    {.actions = {rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbFinish, rmbDFAErr, rmbI2CErr}}, //clear flags after NACK and STOP conditions
  };

  // SB, ADDR, BTF, RxNE, TxE, ERR
  static I2CAsyncState_t statesEQ1[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbI2CErr}}, //init. if some interrupt happens here - it's NOT OK!
    //1
    {.actions = {rmbSendAddrTransmitter, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbI2CErr}}, //start sent. send i2c slave addr (transmitter mode)
    //2
    {.actions = {rmbDFAErr, rmbSendStartReg, rmbDFAErr, rmbDFAErr, rmbDFAErr, rmbI2CErr}}, //addr sent. send read register addr. TxE not set in addr stage
    //3
    {.actions = {rmbSendAddrReceiver, rmbDFAErr, dfaNop, rmbDFAErr, dfaNop, rmbI2CErr}}, //start sent. send i2c slave addr (receiver mode)
    //4
    {.actions = {rmbDFAErr, rmbAddrReceivedLen1, rmbDFAErr, rmbFinish, dfaNop, rmbI2CErr}}, //addr sent. wait for byte received
  };

  rmbCtx.i2c = I2C1;
  rmbCtx.slaveAddr = slaveAddr;
  rmbCtx.dstReg = startReg;
  rmbCtx.dstBuff = dstBuff;
  rmbCtx.readLen = readLen;
  rmbCtx.pFinishCode = pErr;
  rmbCtx.cb = cb;
  res.cs = DFA_INIT_STATE;

  I2CAsyncState_t *states = readLen == 1 ? statesEQ1 : statesGE2;
  size_t len = readLen == 1 ? sizeof(statesEQ1) : sizeof(statesGE2);

  for (size_t i = 0; i < len/sizeof(I2CAsyncState_t); ++i) {
    states[i].arg = &rmbCtx;
    states[i].ix = (int32_t)i;
    res.states[i] = states[i];
  }

  return res;
}
///////////////////////////////////////////////////////


void i2c1_init() {
  I2C_InitTypeDef I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  m_dfaI2C1 = preInitDFA();

  I2C_Cmd(I2C1, DISABLE);
  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_Pin | I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  //  I2C_InitStructure.I2C_OwnAddress1 = MPU6050_DEFAULT_ADDRESS; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  i2c1_interrupts(ENABLE);
}
///////////////////////////////////////////////////////

void i2c1_interrupts(FunctionalState on) {
  NVIC_InitTypeDef nvicInitCfg;
  /* Enable the I2C1 event Interrupt */
  nvicInitCfg.NVIC_IRQChannel = I2C1_EV_IRQn;
  nvicInitCfg.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInitCfg.NVIC_IRQChannelSubPriority = 0;
  nvicInitCfg.NVIC_IRQChannelCmd = on;
  NVIC_Init(&nvicInitCfg);

  /* Enable the I2C1 error interrupt with higher then event priority */
  nvicInitCfg.NVIC_IRQChannel = I2C1_ER_IRQn;
  nvicInitCfg.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInitCfg.NVIC_IRQChannelSubPriority = 0;
  nvicInitCfg.NVIC_IRQChannelCmd = on;
  NVIC_Init(&nvicInitCfg);
  I2C_ITConfig(I2C1, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, on);
}
///////////////////////////////////////////////////////

void i2c1_read_buff_async(uint8_t slaveAddr,
                          uint8_t startReg,
                          uint8_t *buff,
                          uint8_t len,
                          volatile int8_t *finishCode,
                          i2c1_pf_callback cb) {
  *finishCode = FC_IN_PROGRESS;
  m_dfaI2C1 = rmbDFA(slaveAddr, startReg, buff, len, finishCode, cb);  
  rmbGenStart(&m_dfaI2C1.states[0], &m_dfaI2C1);
}
///////////////////////////////////////////////////////

I2C1FinishCode i2c1_read_buff_sync(uint8_t slaveAddr,
                                   uint8_t startReg,
                                   uint8_t *buff,
                                   uint8_t len) {
  volatile int8_t finishCode = FC_IN_PROGRESS;
  i2c1_read_buff_async(slaveAddr, startReg, buff, len, &finishCode, NULL);
  while (finishCode == FC_IN_PROGRESS);
  return (I2C1FinishCode)finishCode;
}
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

static I2CAsynDFA_t wmbDFA(uint8_t slaveAddr,
                           uint8_t startReg,
                           volatile uint8_t *srcBuff,
                           uint8_t writeLen,
                           volatile int8_t *pErr, i2c1_pf_callback cb);
static void wmbGenStart(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void wmbSendAddr(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void wmbSendStartReg(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void wmbSendData(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void wmbFinish(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);

static void wmbDFAErr(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);
static void wmbI2CErr(struct I2CAsyncState *st, struct I2CAsynDFA *dfa);

static void wmbI2CErr_Berr(WMBContext_t *ctx);
static void wmbI2CErr_AF(WMBContext_t *ctx);
static void wmbI2CErr_Arlo(WMBContext_t *ctx);
static void wmbI2CErr_Ovr(WMBContext_t *ctx);
static void wmbI2CErr_Timeout(WMBContext_t *ctx);

void wmbGenStart(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  while (I2C_GetFlagStatus(pctx->i2c, I2C_FLAG_BUSY))
    __NOP();
  dfa->cs = 1;
  I2C_GenerateSTART(pctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void wmbSendAddr(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  I2C_Send7bitAddress(pctx->i2c, pctx->slaveAddr, I2C_Direction_Transmitter);
  dfa->cs = 2;
}
///////////////////////////////////////////////////////

void wmbSendStartReg(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  I2C_SendData(pctx->i2c, pctx->dstReg);
  dfa->cs = 3;
}
///////////////////////////////////////////////////////

void wmbSendData(struct I2CAsyncState *st, struct I2CAsynDFA *dfa) {
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  I2C_SendData(pctx->i2c, *pctx->srcBuff++);
  if (pctx->writeLen-- != 1) {
    dfa->cs = 3;
    return;
  }
  I2C_GenerateSTOP(pctx->i2c, ENABLE);
  dfa->cs = 4;
}
///////////////////////////////////////////////////////

void wmbFinish(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  if (pctx->cb != NULL)
    pctx->cb(FC_FINISHED);
  dfa->cs = DFA_INIT_STATE; //goto init state
  *pctx->pFinishCode = FC_FINISHED;
}
///////////////////////////////////////////////////////

void wmbI2CErr_Berr(WMBContext_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void wmbI2CErr_AF(WMBContext_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void wmbI2CErr_Arlo(WMBContext_t *ctx) {
  UNUSED(ctx);
  //todo reset all slaves
}
///////////////////////////////////////////////////////

void wmbI2CErr_Ovr(WMBContext_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void wmbI2CErr_Timeout(WMBContext_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void wmbI2CErr(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  volatile uint16_t sr1;
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  sr1 = pctx->i2c->SR1;
  pctx->i2c->SR1 = (sr1 & ~I2C_ERR_MASK);

  if (sr1 & I2C_SR1_BERR)
    wmbI2CErr_Berr(pctx);
  if (sr1 & I2C_SR1_AF)
    wmbI2CErr_AF(pctx);
  if (sr1 & I2C_SR1_ARLO)
    wmbI2CErr_Arlo(pctx);
  if (sr1 & I2C_SR1_OVR)
    wmbI2CErr_Ovr(pctx);
  if (sr1 & I2C_SR1_TIMEOUT)
    wmbI2CErr_Timeout(pctx);
  if (pctx->cb != NULL)
    pctx->cb(FC_I2C_ERR);
  *pctx->pFinishCode = FC_I2C_ERR;
  dfa->cs = DFA_INIT_STATE;
}
///////////////////////////////////////////////////////

//todo: we can't be here. SO! just restart device and i2c peripheral
void wmbDFAErr(struct I2CAsyncState* st, struct I2CAsynDFA *dfa) {
  WMBContext_t *pctx = (WMBContext_t*)(st->arg);
  I2C_Cmd(pctx->i2c, DISABLE);
  I2C_Cmd(pctx->i2c, ENABLE);
  if (pctx->cb != NULL)
    pctx->cb(FC_I2C_ERR);
  *pctx->pFinishCode = FC_I2C_ERR;
  dfa->cs = DFA_INIT_STATE;
}
///////////////////////////////////////////////////////

I2CAsynDFA_t wmbDFA(uint8_t slaveAddr,
                    uint8_t startReg,
                    volatile uint8_t *srcBuff,
                    uint8_t writeLen,
                    volatile int8_t *pErr,
                    i2c1_pf_callback cb) {
  I2CAsynDFA_t res;
  static WMBContext_t wmbCtx;

  // SB, ADDR, BTF, RxNE, TxE, ERR
  static I2CAsyncState_t states[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {wmbDFAErr, wmbDFAErr, wmbDFAErr, wmbDFAErr, wmbDFAErr, wmbI2CErr}}, //init
    //1
    {.actions = {wmbSendAddr, wmbDFAErr, wmbDFAErr, wmbDFAErr, wmbDFAErr, wmbI2CErr}}, //start sent. send i2c slave addr (transmitter mode)
    //2
    {.actions = {wmbDFAErr, wmbSendStartReg, wmbDFAErr, wmbDFAErr, wmbDFAErr, wmbI2CErr}}, //addr sent. send write register addr. TxE not set in addr stage
    //3
    {.actions = {wmbDFAErr, dfaNop, wmbSendData, wmbDFAErr, dfaNop, wmbI2CErr}}, //addr sent. wait for byte transmitted
    //4
    {.actions = {wmbDFAErr, wmbDFAErr, wmbFinish, wmbDFAErr, dfaNop, wmbI2CErr}}, //clear flags after NACK and STOP conditions
  };

  wmbCtx.i2c = I2C1;
  wmbCtx.slaveAddr = slaveAddr;
  wmbCtx.dstReg = startReg;
  wmbCtx.srcBuff = srcBuff;
  wmbCtx.writeLen = writeLen;
  wmbCtx.pFinishCode = pErr;
  wmbCtx.cb = cb;
  res.cs = DFA_INIT_STATE;

  for (size_t i = 0; i < sizeof(states)/sizeof(I2CAsyncState_t); ++i) {
    states[i].arg = &wmbCtx;
    states[i].ix = (int32_t)i;
    res.states[i] = states[i];
  }
  return res;
}
///////////////////////////////////////////////////////

void i2c1_write_buff_async(uint8_t slaveAddr,
                           uint8_t startReg,
                           uint8_t *buff,
                           uint8_t len,
                           volatile int8_t *finishCode,
                           i2c1_pf_callback cb) {
  *finishCode = FC_IN_PROGRESS;
  m_dfaI2C1 = wmbDFA(slaveAddr, startReg, buff, len, finishCode, cb);
  wmbGenStart(&m_dfaI2C1.states[0], &m_dfaI2C1);
}
///////////////////////////////////////////////////////

I2C1FinishCode i2c1_write_buff_sync(uint8_t slaveAddr,
                                    uint8_t startReg,
                                    uint8_t *buff,
                                    uint8_t len) {
  volatile int8_t finishCode = FC_IN_PROGRESS;
  i2c1_write_buff_async(slaveAddr, startReg, buff, len, &finishCode, NULL);
  while (finishCode == FC_IN_PROGRESS);
  return (I2C1FinishCode) finishCode;
}
///////////////////////////////////////////////////////

void i2c1_scan(void (*cb)(uint8_t, uint8_t)) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 0x7f; ++i) {
    uint8_t tmp = 0;
    I2C1FinishCode err = i2c1_read_buff_sync(i << 1, 0, &tmp, 1);
    if (err == FC_FINISHED) {
      cb(i, count++);
    }
  }
}
///////////////////////////////////////////////////////
