/*
 * mpx.c
 *
 *  Created on: Dec 5, 2024
 *      Author: Asela Fernando
 *
 */
#include <string.h>
#include "main.h"
#include "mpx.h"

#ifndef  MPX_TIM_HANDLE
#error "MPX_TIM_HANDLE not defined!"
#endif

#ifndef MPX_CLK
#error "MPX_CLK not defined"
#endif

#if !defined(MPX_TX_GPIO_Port) || !defined(MPX_TX_Pin)
#error "Definitions of MPX_TX_GPIO_Port or MPX_TX_Pin missing!"
#endif

#if !defined(MPX_RX_GPIO_Port) || !defined(MPX_RX_Pin)
#error "Definitions of MPX_RX_GPIO_Port or MPX_RX_Pin missing!"
#endif

#define MPX_TX(v) 	  HAL_GPIO_WritePin(MPX_TX_GPIO_Port, MPX_TX_Pin, v)
#define MPX_TX_READ() HAL_GPIO_ReadPin(MPX_TX_GPIO_Port, MPX_TX_Pin)
#define MPX_RX() 	  HAL_GPIO_ReadPin(MPX_RX_GPIO_Port, MPX_RX_Pin)

#define MPX_BAUD_RATE       10000
#define MPX_MAX_PACKET_SIZE 15      // PRI + ML + DST-ID + MSG-ID + DATA
#define MPX_HEADER_SIZE    	2
#define MPX_MAX_DATA_SIZE  	11
#define MPX_MAX_FRAME_SIZE  (MPX_HEADER_SIZE + MPX_MAX_DATA_SIZE + 1 + 1)
#define MPX_QUEUE_SIZE  	16

#ifdef MPX_TIM_OC_CHAN
#define full_bit_timer() __HAL_TIM_SET_COMPARE(&MPX_TIM_HANDLE, MPX_TIM_OC_CHAN, MPX_CLK / MPX_BAUD_RATE - 1)
#define half_bit_timer() __HAL_TIM_SET_COMPARE(&MPX_TIM_HANDLE, MPX_TIM_OC_CHAN, MPX_CLK / MPX_BAUD_RATE / 2 - 1)
#else
#define full_bit_timer() __HAL_TIM_SET_AUTORELOAD(&MPX_TIM_HANDLE, F_CLK / MPX_CLK / MPX_BAUD_RATE - 1)
#define half_bit_timer() __HAL_TIM_SET_AUTORELOAD(&MPX_TIM_HANDLE, F_CLK / MPX_CLK / MPX_BAUD_RATE / 2 - 1)
#endif

#ifndef MPX_TX_LED_OFF
#define MPX_TX_LED_OFF() __NOP()
#endif
#ifndef MPX_TX_LED_ON
#define MPX_TX_LED_ON()  __NOP()
#endif
#ifndef MPX_RX_LED_OFF
#define MPX_RX_LED_OFF() __NOP()
#endif
#ifndef MPX_RX_LED_ON
#define MPX_RX_LED_ON()  __NOP()
#endif

static uint8_t queue_head = 0;     // queue write index
static uint8_t queue_tail = 0;     // queue read index
static uint8_t queue[MPX_QUEUE_SIZE][MPX_MAX_PACKET_SIZE];
extern TIM_HandleTypeDef MPX_TIM_HANDLE;

enum
{
  TX_IDLE = 0,
  TX_QUEUED,
  TX_START,
  TX_IN_PROGRESS,
  TX_ARBITRATION,
  TX_ACKNOWLEGEMENT
};

//-----------------------------------------------------------------------------
static struct
{
  bool idle;
} bus;
//-----------------------------------------------------------------------------
static struct
{
  uint8_t buf[MPX_MAX_FRAME_SIZE];
  uint8_t size;
  uint8_t crc;
  volatile uint8_t byte;
  uint8_t bit;
  bool busy;
  uint8_t same_bit_counter;
  void (*callback)(uint8_t size, const uint8_t* buf);
} rx;
//-----------------------------------------------------------------------------
static struct
{
  uint8_t buf[MPX_MAX_FRAME_SIZE];
  uint8_t size;
  uint8_t pos;
  uint8_t shift;
  volatile uint8_t state;
  uint8_t result;
  void (*callback)(uint8_t result);
} tx;
//-----------------------------------------------------------------------------
static struct
{
  uint8_t counter;
  uint8_t prev;
} stuffing;

#ifdef MPX_CRC_HANDLE
extern CRC_HandleTypeDef MPX_CRC_HANDLE;
#define MPX_CRC8(data, size) HAL_CRC_Calculate(&MPX_CRC_HANDLE, (uint32_t*)data, size)
#else
// CRC: Polynomial = 8X + 4X + X + 1
const uint8_t CRC8_table[256] = {
  0x00, 0x13, 0x26, 0x35, 0x4C, 0x5F, 0x6A, 0x79, 0x98, 0x8B, 0xBE, 0xAD, 0xD4, 0xC7, 0xF2, 0xE1,
  0x23, 0x30, 0x05, 0x16, 0x6F, 0x7C, 0x49, 0x5A, 0xBB, 0xA8, 0x9D, 0x8E, 0xF7, 0xE4, 0xD1, 0xC2,
  0x46, 0x55, 0x60, 0x73, 0x0A, 0x19, 0x2C, 0x3F, 0xDE, 0xCD, 0xF8, 0xEB, 0x92, 0x81, 0xB4, 0xA7,
  0x65, 0x76, 0x43, 0x50, 0x29, 0x3A, 0x0F, 0x1C, 0xFD, 0xEE, 0xDB, 0xC8, 0xB1, 0xA2, 0x97, 0x84,
  0x8C, 0x9F, 0xAA, 0xB9, 0xC0, 0xD3, 0xE6, 0xF5, 0x14, 0x07, 0x32, 0x21, 0x58, 0x4B, 0x7E, 0x6D,
  0xAF, 0xBC, 0x89, 0x9A, 0xE3, 0xF0, 0xC5, 0xD6, 0x37, 0x24, 0x11, 0x02, 0x7B, 0x68, 0x5D, 0x4E,
  0xCA, 0xD9, 0xEC, 0xFF, 0x86, 0x95, 0xA0, 0xB3, 0x52, 0x41, 0x74, 0x67, 0x1E, 0x0D, 0x38, 0x2B,
  0xE9, 0xFA, 0xCF, 0xDC, 0xA5, 0xB6, 0x83, 0x90, 0x71, 0x62, 0x57, 0x44, 0x3D, 0x2E, 0x1B, 0x08,
  0x0B, 0x18, 0x2D, 0x3E, 0x47, 0x54, 0x61, 0x72, 0x93, 0x80, 0xB5, 0xA6, 0xDF, 0xCC, 0xF9, 0xEA,
  0x28, 0x3B, 0x0E, 0x1D, 0x64, 0x77, 0x42, 0x51, 0xB0, 0xA3, 0x96, 0x85, 0xFC, 0xEF, 0xDA, 0xC9,
  0x4D, 0x5E, 0x6B, 0x78, 0x01, 0x12, 0x27, 0x34, 0xD5, 0xC6, 0xF3, 0xE0, 0x99, 0x8A, 0xBF, 0xAC,
  0x6E, 0x7D, 0x48, 0x5B, 0x22, 0x31, 0x04, 0x17, 0xF6, 0xE5, 0xD0, 0xC3, 0xBA, 0xA9, 0x9C, 0x8F,
  0x87, 0x94, 0xA1, 0xB2, 0xCB, 0xD8, 0xED, 0xFE, 0x1F, 0x0C, 0x39, 0x2A, 0x53, 0x40, 0x75, 0x66,
  0xA4, 0xB7, 0x82, 0x91, 0xE8, 0xFB, 0xCE, 0xDD, 0x3C, 0x2F, 0x1A, 0x09, 0x70, 0x63, 0x56, 0x45,
  0xC1, 0xD2, 0xE7, 0xF4, 0x8D, 0x9E, 0xAB, 0xB8, 0x59, 0x4A, 0x7F, 0x6C, 0x15, 0x06, 0x33, 0x20,
  0xE2, 0xF1, 0xC4, 0xD7, 0xAE, 0xBD, 0x88, 0x9B, 0x7A, 0x69, 0x5C, 0x4F, 0x36, 0x25, 0x10, 0x03};

static uint8_t MPX_CRC8(const uint8_t* data, uint8_t size)
{
  uint8_t crc = 0;
  while(size--)
  {
    crc = CRC8_table[crc ^ *data++];
  }
  return crc;
}
#endif

//-----------------------------------------------------------------------------
static inline void stop_timer(void)
{
  MPX_TIM_HANDLE.Instance->CR1 &= ~TIM_CR1_CEN;
}

static inline void start_timer(void)
{
    __HAL_TIM_SET_COUNTER(&MPX_TIM_HANDLE, 0);
  MPX_TIM_HANDLE.Instance->CR1 |= TIM_CR1_CEN;
}

static inline void resync_timer(void)
{
  __HAL_TIM_SET_COUNTER(&MPX_TIM_HANDLE, 0);
  half_bit_timer();
  MPX_TIM_HANDLE.Instance->CR1 |= TIM_CR1_CEN;
}
/**
 * @brief Checks whether the stuffing bit is needed after the previous push for
 * transmission
 * @return 1 - stuffing bit should be the next
 */
static inline bool stuffing_check(void)
{
  return (stuffing.counter >= 5);
}

static inline void stuffing_reset(void)
{
  stuffing.counter = 0;
}
/**
 * @brief Checks whether the driver controls the MPX bus during transmission
 * @return true - The driver has lost the control of the bus, another device is
 * transmitting a message with a greater priority or higher message-id
 */
static inline bool arbitration_lost()
{
  return (!MPX_TX_READ() && MPX_RX());
}
/**
 * @brief Registers the bit and determines whether it is a stuffing bit
 * @param value - bit value
 * @return 1 - the bit is a stuffing bit and should be skipped in reception
 */
static inline uint8_t stuffing_push(uint8_t value)
{
  value = (value > 0) ? 1 : 0;
  if(value == stuffing.prev) {
    stuffing.counter++;
    return 0;
  }
  volatile uint8_t res = 0;
  if(stuffing.counter == 5) {
    // stuffing bit detected
    res = 1;
  }
  stuffing.counter = 1;
  stuffing.prev = value;
  return res;
}

/**
 * @brief Returns the value (polarity) of the following stuffing bit
 */
static inline uint8_t stuffing_bit()
{
  stuffing.counter = 1;
  stuffing.prev = (stuffing.prev > 0) ? 0 : 1;  // stuffing bit is an inverted bit
  return stuffing.prev;
}

static inline void rx_reset()
{
  rx.size = 0;
  rx.crc = 0;
  rx.byte = 0;
  rx.bit = 0;
  stuffing_reset();
}

static inline void suspend_transmission()
{
  // suspend the transmission
  MPX_TX(GPIO_PIN_RESET);

  tx.state = TX_QUEUED;
  MPX_TX_LED_OFF();

  //and continue reception
  MPX_RX_LED_ON();
  rx.busy = true;

  // restore skipped data
  volatile register uint8_t size = tx.pos;
  memcpy(rx.buf, tx.buf, size);
  rx.crc = MPX_CRC8(rx.buf, size);
  rx.bit = tx.shift - 1;
  rx.byte = tx.buf[size] >> (9 - tx.shift);
  rx.size = size;
}

static inline void start_transmission()
{
  if(!MPX_RX())
  {
    // start of frame (a start bit)
    tx.pos = 0;
    tx.shift = 0;
    stuffing_reset();
    stuffing_push(1);
    tx.state = TX_IN_PROGRESS;
    MPX_TX(GPIO_PIN_SET);
    full_bit_timer();
    start_timer();
  }
}

static inline void get_bit()
{
  static uint8_t EOM = 0;
  EOM = EOM << 1 | MPX_RX();
  if(EOM == 0x7E)
  {
#ifdef MPX_CRC_HANDLE
    rx.crc = MPX_CRC8(rx.buf, rx.size);
#endif
    // end-of-message detected
    rx.buf[rx.size++] = 0x7E;

    if(!rx.crc)
      {
	// if received CRC and calculated CRC are equal, then
	// the result CRC should be zero
	if(rx.callback)
	  {
	    rx.callback(rx.size, rx.buf);
	  }
      }
    rx_reset();
    return;
  }

  if(!rx.busy && MPX_RX())
  {
    // start bit detected
    MPX_RX_LED_ON();
    rx_reset();
    rx.busy = true;
    return;
  }

  if(stuffing_push(MPX_RX()))
  {
    // skip stuffing-bit
    return;
  }

  rx.byte = rx.byte << 1 | MPX_RX();
  if(++rx.bit >= 8)
  {
    // byte reception completed
    rx.bit = 0;
    rx.buf[rx.size++] = rx.byte;
#ifndef MPX_CRC_HANDLE
    //Calculate CRC on the fly
    rx.crc = CRC8_table[rx.crc ^ rx.byte];
#endif
  }
}

static inline void send_bit()
{
  volatile register uint8_t value = 0;

  if(tx.pos < tx.size && stuffing_check())
  {
    if(stuffing_bit())
      MPX_TX(GPIO_PIN_SET);
    else
      MPX_TX(GPIO_PIN_RESET);
    return;
  }

  if(tx.pos <= tx.size)
  {
    // various data (from priority to CRC)
    value = (tx.buf[tx.pos] << tx.shift) & 0x80;
    // set bit value as soon as possible
    if(value)
    {
      MPX_TX(GPIO_PIN_SET);
    }
    else
    {
      MPX_TX(GPIO_PIN_RESET);
    }
    if(++tx.shift >= 8)
    {
      tx.pos++;
      tx.shift = 0;
    }
    if(value)
    {
      MPX_TX(GPIO_PIN_SET);
      stuffing_push(1);
    }
    else
    {
      MPX_TX(GPIO_PIN_RESET);
      // prepare arbitration check
      half_bit_timer();
      tx.state = TX_ARBITRATION;
    }
  }
  else
  {
    switch(tx.shift++)
    {
      case 0:
	// prepare to receive checksum and acknowlegement reply
	resync_timer();
	tx.state = TX_ACKNOWLEGEMENT;
	// prevent detecting acknowlegement as a start bit
	rx.busy = true;
	break;
      case 1:
	// checksum reply
	tx.result = MPX_RX() << 1;
	break;
      case 2:
	// acknowlegement reply
	tx.result = (tx.result | MPX_RX());
	break;
      case 3:
	// resync back into transmission mode
	resync_timer();
	if(!MPX_RX())
	{
	  tx.result = tx.result << 1 | 1;
	  // tx.result values
	  // "001" = 1 -- NACK
	  // "011" = 3 -- ACK
	  // "111" = 7 -- BAD CRC
	}
	else
	{
	  // unexpected bus level
	  tx.result = MPX_ERR_UNKNOWN;
	}

	// transmission completed
	MPX_TX_LED_OFF();
	tx.state = TX_IDLE;
	if(tx.callback)
	{
	  tx.callback(tx.result);
	}
	rx.busy = false;
	break;
    }
  }
}

uint8_t mpx_send(uint8_t priority, uint8_t address, uint8_t size, const uint8_t* data, void (*callback)(uint8_t result))
{
  if(tx.state != TX_IDLE)
  {
    return MPX_ERR_BUSY;
  }
  if((priority > 15) || (size < 2) || (size > MPX_MAX_DATA_SIZE + 1))
  {
    // valid priority range: 0...15
    // valid size range: 2...12 (includes message id)
    return MPX_ERR_BAD_PARAMETER;
  }
  // priority and size
  tx.buf[0] = (priority << 4) | (size + 1);

  // destination address
  tx.buf[1] = address;

  // data
  memcpy(tx.buf + 2, data, size);

  // CRC
  tx.buf[size + 2] = MPX_CRC8(tx.buf, size + 2);

  // EOM
  tx.buf[size + 3] = 0x7E;

  tx.size = size + 3;
  tx.callback = callback;
  tx.state = TX_QUEUED;

  if(bus.idle)
  {
    start_transmission();
  }
  return MPX_ERR_QUEUED;
}

static inline bool mpx_queue_is_empty(void)
{
  return (queue_head == queue_tail);
}

static inline bool mpx_queue_is_not_empty(void)
{
  return (queue_head != queue_tail);
}

void mpx_queue_check(uint8_t result)
{
  UNUSED(result);

  if(mpx_queue_is_not_empty())
  {
    uint8_t index = queue_tail;
    queue_tail = (queue_tail + 1) % MPX_QUEUE_SIZE;
    mpx_send(queue[index][0], queue[index][1], queue[index][2], queue[index] + 3, mpx_queue_check);
  }
}

uint8_t mpx_queue(uint8_t priority, uint8_t address, uint8_t size, const uint8_t* data)
{
  uint8_t result = 0;
  if(mpx_queue_is_empty() && tx.state == TX_IDLE)
  {
    result = mpx_send(priority, address, size, data, mpx_queue_check);
  }
  else
  {
    uint8_t next_index = (queue_head + 1) % MPX_QUEUE_SIZE;
    if(next_index == queue_tail)
    {
      // step on self tail
      result = MPX_ERR_QUEUE_OVERFLOW;
    }
    else
    {
      queue[queue_head][0] = priority;
      queue[queue_head][1] = address;
      queue[queue_head][2] = size;
      memcpy(queue[queue_head] + 3, data, size);
      queue_head = next_index;
    }
  }
  return result;
}

inline void mpx_rx_interrupt(void)
{
  // resync the timer
  if(tx.state == TX_IN_PROGRESS)      // do not resync while transmission is in progress
  {
    full_bit_timer();
    start_timer();
  }
  else
  {
    resync_timer();
  }
  bus.idle = false;
  if(tx.state == TX_IN_PROGRESS && arbitration_lost())
  {
    //MPX_HW_DEBUG_SYNC();
    resync_timer();
    suspend_transmission();
  }
  // continue further receiption
  rx.same_bit_counter = 0;
}

inline void mpx_timer_interrupt(void)
{
  // change the timer interval to full-bit value
  full_bit_timer();
  switch(tx.state)
  {
    case TX_ARBITRATION:
      if(arbitration_lost())
      {
	    suspend_transmission();
	    get_bit();
      }
      else
      {
	    // arbitration is OK, continue the transmission
	    stuffing_push(0);
	    tx.state = TX_IN_PROGRESS;
	    half_bit_timer();
      }
      break;
    case TX_IN_PROGRESS:
    case TX_ACKNOWLEGEMENT:
      // transmission mode
      send_bit();
      break;
    default:
      // reception mode
      if(++rx.same_bit_counter > 6)
      {
	    // idle state detected
	    bus.idle = true;
	    rx.busy = false;
	    MPX_RX_LED_OFF();
	    rx.same_bit_counter = 0;
	    if(tx.state == TX_QUEUED)
	    {
	      // a packet is waiting for transmission
	      start_transmission();
	    }
	    else
	    {
	      // go to idle mode
	      stop_timer();
	    }
      }
      else
      {
	    get_bit();
      }
  }
}

void mpx_init(void (*rx_callback)(uint8_t size, const uint8_t* buf))
{
  rx.callback = rx_callback;
  bus.idle = true;
  rx.busy = false;
  tx.state = TX_IDLE;
  stuffing.counter = 0;
  stuffing.prev = 0;

  //Start timer using HAL which sets up all the registers and callbacks
#ifdef MPX_TIM_OC_CHAN
  HAL_TIM_OC_Start_IT(&MPX_TIM_HANDLE, MPX_TIM_OC_CHAN);
#else
  HAL_TIM_Base_Start_IT(&MPX_TIM_HANDLE);
#endif
  //Disable Timer counting
  MPX_TIM_HANDLE.Instance->CR1 &= ~TIM_CR1_CEN;
  //Reset the count back to 0
  __HAL_TIM_SET_COUNTER(&MPX_TIM_HANDLE, 0);
  // MPX_RX_Pin IOC, rising and calling tokeep in sync, correcting the timer on every level change
}
