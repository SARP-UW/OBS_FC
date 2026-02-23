/**
 * This file is part of the Titan Flight Computer Project
 * Copyright (c) 2024 UW SARP
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file peripheral/uart.c
 * @authors Charles Faisandier, Joanna Zhou, Jude Merritt
 * @brief USART Driver Implementation
 */

#include "uart.h"
#include "../internal/mmio.h"
#include "gpio.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <internal/mmio.h>

#define CR_REG_COUNT 3
#define NUM_REQUESTS_PER_UART 2

#define IS_USART_CHANNEL(channel)                                              \
  ((channel) == UART1 || (channel) == UART2 || (channel) == UART3 ||           \
   (channel) == UART6)

/**************************************************************************************************
 * @section  Data Structures
 **************************************************************************************************/

// Stores dmamux request numbers. Index 0 is for RX stream, 1 for TX stream.
const static uint8_t uart_dmamux_req[UART_CHANNEL_COUNT]
                                    [NUM_REQUESTS_PER_UART] = {
                                        [UART1] =
                                            {
                                                [0] = 41,
                                                [1] = 42,
                                            },
                                        [UART2] =
                                            {
                                                [0] = 43,
                                                [1] = 44,
                                            },
                                        [UART3] =
                                            {
                                                [0] = 45,
                                                [1] = 46,
                                            },
                                        [UART4] =
                                            {
                                                [0] = 63,
                                                [1] = 64,
                                            },
                                        [UART5] =
                                            {
                                                [0] = 65,
                                                [1] = 66,
                                            },
                                        [UART6] =
                                            {
                                                [0] = 71,
                                                [1] = 72,
                                            },
                                        [UART7] =
                                            {
                                                [0] = 79,
                                                [1] = 80,
                                            },
                                        [UART8] =
                                            {
                                                [0] = 81,
                                                [1] = 82,
                                            },
};

volatile dma_periph_streaminfo_t uart_to_dma[UART_CHANNEL_COUNT] = {0};

bool uart_busy[UART_CHANNEL_COUNT] = {0};

uart_context_t uart_contexts[UART_CHANNEL_COUNT] = {0};

uint32_t timeout;

/**************************************************************************************************
 * @section Private Function Implementations
 **************************************************************************************************/
bool set_alternate_function(uart_channel_t channel, uint8_t tx_pin,
                            uint8_t rx_pin, uint8_t ck_pin) {
  switch (channel) {
    case UART1:
      // if (tx_pin == 74) {
      //   tal_alternate_mode(tx_pin, 3);
      // } else if (tx_pin == 98) {
      //   tal_alternate_mode(tx_pin, 5);
      // } else if (tx_pin == 133) {
      //   tal_alternate_mode(tx_pin, 6);
      // } else {
      //   return false;
      // }
      if (tx_pin == 98 || tx_pin == 133) {
        tal_alternate_mode(tx_pin, 7);
      } else if (tx_pin == 74) {
        tal_alternate_mode(tx_pin, 4);
      } else {
        // // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }

      // if (rx_pin == 75) {
      //   tal_alternate_mode(rx_pin, 4);
      // } else if (rx_pin == 99) {
      //   tal_alternate_mode(rx_pin, 3);
      // } else if (rx_pin == 134) {
      //   tal_alternate_mode(rx_pin, 5);
      // } else {
      //   return false;
      // }
      if (rx_pin == 99 || rx_pin == 134) {
        tal_alternate_mode(rx_pin, 7);
      } else if (rx_pin == 75) {
        tal_alternate_mode(rx_pin, 4);
      } else {
        // // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }
      // if (ck_pin == 97) {
      //   tal_alternate_mode(ck_pin, 5);
      // } else {
      //   return false;
      // }

      break;
    case UART2:
      // if (tx_pin == 39) {
      //   tal_alternate_mode(tx_pin, 4);
      // } else if (tx_pin == 117) {
      //   tal_alternate_mode(tx_pin, 1);
      // } else {
      //   return false;
      // }
      if (tx_pin == 39 || tx_pin == 117) {
        tal_alternate_mode(tx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }

      // if(rx_pin == 40) {
      //   tal_alternate_mode(rx_pin, 4);
      // } else if (rx_pin == 120) {
      //   tal_alternate_mode(rx_pin, 5);
      // } else {
      //   return false;
      // }
      if (rx_pin == 40 || rx_pin == 120) {
        tal_alternate_mode(rx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }

      if (ck_pin == 43) {
        tal_alternate_mode(ck_pin, 4);
      } else if (ck_pin == 121) {
        tal_alternate_mode(ck_pin, 3);
      }else {
        return false;
      }
      break;
    case UART3:
      // if (tx_pin == 66) {
      //   tal_alternate_mode(tx_pin, 6);
      // } else if (tx_pin == 76) {
      //   tal_alternate_mode(tx_pin, 2);
      // } else if (tx_pin == 109) {
      //   tal_alternate_mode(tx_pin, 3);
      // } else {
      //   return false;
      // }
      if (tx_pin == 66 || tx_pin == 109 || tx_pin == 76) {
        tal_alternate_mode(tx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }
      // if(rx_pin == 67) {
      //   tal_alternate_mode(rx_pin, 5);
      // } else if (rx_pin == 77) {
      //   tal_alternate_mode(rx_pin, 2);
      // } else if (rx_pin == 110) {
      //   tal_alternate_mode(rx_pin, 3);
      // } else {
      //   return false;
      // }
      if (rx_pin == 67 || rx_pin == 110 || rx_pin == 77) {
        tal_alternate_mode(rx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }
      if (ck_pin == 72) {
        tal_alternate_mode(ck_pin, 4);
      } else if (ck_pin == 78) {
        tal_alternate_mode(ck_pin, 2);
      } else if (ck_pin == 111) {
        tal_alternate_mode(ck_pin, 3);
      } else {
        return false;
      }
      break;
    case UART4:
      // TODO: what the HECK does PA0_C mean??
      if (tx_pin == 37 || tx_pin == 137 || tx_pin == 109 || tx_pin == 113) {
        tal_alternate_mode(tx_pin, 8);
      } else if (tx_pin == 101) {
        tal_alternate_mode(tx_pin, 6);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }
      // if (tx_pin == 101 || tx_pin == 109) {
      //   tal_alternate_mode(tx_pin, 4);
      // } else if (tx_pin == 0) {
      //   tal_alternate_mode(tx_pin, 1);
      // }  else if (tx_pin == 112) {
      //   tal_alternate_mode(tx_pin, 2);
      // } else if (tx_pin == 137) {
      //   tal_alternate_mode(tx_pin, 7);
      // } else {
      //   return false;
      // }
      if (rx_pin == 38 || rx_pin == 137 || rx_pin == 110 || rx_pin == 112) {
        tal_alternate_mode(rx_pin, 8);
      } else if (rx_pin == 100) {
        tal_alternate_mode(rx_pin, 6);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }

      // TODO: what does PA1_C mean
      // if (rx_pin == 0) {
      //   // tal_alternate_mode(rx_pin, 0); // TODO: I9 is alt mode 0, H14 alt mode 1??
      // } else if (rx_pin == 100 || rx_pin == 110) {
      //   tal_alternate_mode(rx_pin, 4);
      // } else if (rx_pin == 112) {
      //   tal_alternate_mode(rx_pin, 2);
      // } else if (rx_pin == 136) {
      //   tal_alternate_mode(rx_pin, 6);
      // } else {
      //   return false;
      // }
      break;
    case UART5:
      if (tx_pin == 133 || tx_pin == 73) {
        tal_alternate_mode(tx_pin, 14);
      } else if (tx_pin == 111) {
        tal_alternate_mode(tx_pin, 8);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }
      if (rx_pin == 132 || rx_pin == 72) {
        tal_alternate_mode(rx_pin, 14);
      } else if (rx_pin == 114) {
        tal_alternate_mode(rx_pin, 8);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }

      // if (tx_pin == 73) {
      //   tal_alternate_mode(tx_pin, 8);
      // } else if (tx_pin == 111) {
      //   tal_alternate_mode(tx_pin, 4);
      // } else if (tx_pin == 133) {
      //   tal_alternate_mode(tx_pin, 13);
      // } else {
      //   return false;
      // }


      // if (rx_pin == 72) {
      //   tal_alternate_mode(rx_pin, 10);
      // } else if (rx_pin ==114 ) {
      //   tal_alternate_mode(rx_pin, 2);
      // } else if (rx_pin == 132) {
      //   tal_alternate_mode(rx_pin, 13);
      // } else {
      //   return false;
      // }
      break;
    case UART6:
      // if (tx_pin == 93) {
      //   tal_alternate_mode(tx_pin, 5);
      // } else if (tx_pin == 127) {
      //   tal_alternate_mode(tx_pin, 3);
      // } else {
      //   return false;
      // }
      if (tx_pin == 93 || tx_pin == 122) {
        tal_alternate_mode(tx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }
      if (rx_pin == 94) {
        // tal_alternate_mode(rx_pin, 6);
        tal_alternate_mode(rx_pin, 7);
      } else if (rx_pin == 122) {
        tal_alternate_mode(rx_pin, 1);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }

      if (ck_pin == 87) {
        tal_alternate_mode(ck_pin, 2);
      } else if (ck_pin == 95) {
        tal_alternate_mode(ck_pin, 4);
      } else {
        return false;
      }
      break;
    case UART7:
      if (tx_pin == 108 || tx_pin == 131) {
        tal_alternate_mode(tx_pin, 11);
      } else if (tx_pin == 58 || tx_pin == 21) {
        tal_alternate_mode(tx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }

      // if (tx_pin == 21) {
      //   tal_alternate_mode(tx_pin, 3);
      // } else if (tx_pin == 58) {
      //   tal_alternate_mode(tx_pin, 2);
      // } else if (tx_pin == 108) { // PA15(JTDI) = ?
      //   tal_alternate_mode(tx_pin, 8);
      // } else if (tx_pin == 131) { // PB4(NJTRST)= ?
      //   tal_alternate_mode(tx_pin, 9);
      // } else {
      //   return false;
      // }
      if (rx_pin == 97 || rx_pin == 130) {
        tal_alternate_mode(rx_pin, 11);
      } else if (rx_pin == 57 || rx_pin == 20) {
        tal_alternate_mode(rx_pin, 7);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }

      // if (rx_pin == 20) {
      //   tal_alternate_mode(rx_pin, 3);
      // } else if (rx_pin == 57) {
      //   tal_alternate_mode(rx_pin, 2);
      // } else if (rx_pin == 97) {
      //   tal_alternate_mode(rx_pin, 7);
      // } else if (rx_pin == 130) { // PB3(JTDO/TRACESWO) = ?
      //   tal_alternate_mode(rx_pin, 8);
      // } else {
      //   return false;
      // }
      break;
    case UART8:
      // if (tx_pin == 0 || tx_pin == 139) {
      //   tal_alternate_mode(tx_pin, 2);
      // } else {
      //   return false;
      // }

      // if (rx_pin == 0) {
      //   tal_alternate_mode(rx_pin, 2);
      // } else if (rx_pin == 138) {
      //   tal_alternate_mode(rx_pin,4);
      // } else {
      //   return false;
      // }
      if (tx_pin == 139) {
        tal_alternate_mode(tx_pin, 8);
      } else {
        // tal_raise(flag, "Invalid TX Pin for channel");
        return false;
      }
      if (rx_pin == 138) {
        tal_alternate_mode(rx_pin, 8);
      } else {
        // tal_raise(flag, "Invalid RX Pin for channel");
        return false;
      }
      break;
    default:
      return false;
  }

  return true;
}

bool uart_write_byte(uart_channel_t channel, uint8_t data) {
  uint32_t count = 0;

  if (IS_USART_CHANNEL(channel)) {
    // Wait until the receive FIFO is not empty.
    while (READ_FIELD(USARTx_ISR[channel], USARTx_ISR_TXE) == 0) {
      if (count++ >= 1000000000) {
        return false; // Return false on timeout
      }
    }
    WRITE_FIELD(USARTx_TDR[channel], USARTx_TDR_TDR, data);

    // This is a blocking function, so we return immediately after the data is
    // placed in the FIFO. If you needed to ensure the data was completely sent,
    // you would wait for the TC (Transmission Complete) flag.
    while (READ_FIELD(USARTx_ISR[channel], USARTx_ISR_TC) == 0) {
      asm("nop");
    }
  } else {
    while (READ_FIELD(UARTx_ISR[channel], UARTx_ISR_TXE) == 0) {
      if (count++ >= 1000000000) {
        return false; // Return false on timeout
      }
    }
     WRITE_FIELD(UARTx_TDR[channel], UARTx_TDR_TDR, data);

    // This is a blocking function, so we return immediately after the data is
    // placed in the FIFO. If you needed to ensure the data was completely sent,
    // you would wait for the TC (Transmission Complete) flag.
    while (READ_FIELD(UARTx_ISR[channel], UARTx_ISR_TC) == 0) {
      asm("nop");
    }
  }

  return true;
}

bool uart_read_byte(uint8_t channel, uint8_t *data) {
  uint32_t count = 0;

  // Input validation: ensure the destination pointer is not NULL
  if (data == NULL) {
    return false;
  }

  if (IS_USART_CHANNEL(channel)) {
    // Wait until the receive FIFO is not empty.
    while (READ_FIELD(USARTx_ISR[channel], USARTx_ISR_RXNE) == 0) {
      asm("nop");
    }
    *data = (uint8_t)READ_FIELD(USARTx_RDR[channel], USARTx_RDR_RDR);
    // while (READ_FIELD(USARTx_ISR[channel], USARTx_ISR_RXNE) == 0) {
    //   asm("nop");
    // }
  } else {
    while (READ_FIELD(UARTx_ISR[channel], UARTx_ISR_RXNE) == 0) {
      asm("nop");
    }
    *data = (uint8_t)READ_FIELD(UARTx_RDR[channel], UARTx_RDR_RDR);
    // while (READ_FIELD(UARTx_ISR[channel], UARTx_ISR_RXNE) == 0) {
    //   asm("nop");
    // }
  }

  // Read the data from the receive data register.
  // The hardware automatically retrieves the next available byte from the FIFO.


  return true;
}

static inline bool verify_transfer_parameters(uart_channel_t channel, uint8_t *buff,
                                       size_t size) {

  if (channel == 0) { // USED TO BE: channel == ((void*) (0)
    return false;
  }
  if (buff == 0) { // USED TO BE: channel == ((void*) (0)
    // tal_raise(flag, "Buffer cannot be NULL");
    return false;
  }
  if (size == 0) {
    // tal_raise(flag, "Size cannot be zero");
    return false;
  }
  return true;
}

/**
 * Macro that generates cases for uart init
 * @author Owen Voskuhl Hayes, Lorde of the Isle, first of his name.
 *
 */
#define UART_FIELD_GENERATOR(uart, shift, reg)                                 \
  case uart: {                                                                 \
    field32_t n;                                                               \
    n.msk = (1 << shift);                                                      \
    n.pos = shift;                                                             \
    SET_FIELD(reg, n);                                                         \
    break;                                                                     \
  }

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/
bool uart_init(uart_config_t *usart_config, dma_callback_t *callback,
               periph_dma_config_t *tx_stream, periph_dma_config_t *rx_stream) {
  // De-reference struct members for readability
  uart_channel_t channel = usart_config->channel;
  uart_parity_t parity = usart_config->parity;
  uart_datalength_t data_length = usart_config->data_length;
  uint8_t tx_pin;
  uint8_t rx_pin;
  uint8_t ck_pin = 0;
  uint32_t baud_rate = usart_config->baud_rate;
  // TODO: I think to get exact numbers for this I need devboard
  uint32_t clk_freq = usart_config->clk_freq;

  // Enable usart clock
  switch (channel) {
    case UART1:
      tx_pin = 98;
      rx_pin = 99;
      ck_pin = 97;
      break;
    case UART2:
      tx_pin = 39;
      rx_pin = 40;
      ck_pin = 43;
      break;
    case UART3:
      tx_pin = 66;
      rx_pin = 67;
      ck_pin = 111;
      break;
    case UART4:
      tx_pin = 37;
      rx_pin = 38;
      break;
    case UART5:
      tx_pin = 133;
      rx_pin = 132;
      break;
    case UART6:
      tx_pin = 93;
      rx_pin = 94;
      ck_pin = 95;
      break;
    case UART7:
      tx_pin = 58;
      rx_pin = 57;
      break;
    case UART8:
      tx_pin = 139;
      rx_pin = 138;
      break;
    default:
      return false;
  }
  switch (channel) {
    UART_FIELD_GENERATOR(UART1, 4, RCC_APB2ENR)
    UART_FIELD_GENERATOR(UART2, 17, RCC_APB1LENR)
    UART_FIELD_GENERATOR(UART3, 18, RCC_APB1LENR)
    UART_FIELD_GENERATOR(UART4, 19, RCC_APB1LENR)
    UART_FIELD_GENERATOR(UART5, 20, RCC_APB1LENR)
    UART_FIELD_GENERATOR(UART6, 5, RCC_APB2ENR)
    UART_FIELD_GENERATOR(UART7, 30, RCC_APB1LENR)
    UART_FIELD_GENERATOR(UART8, 31, RCC_APB1LENR)
    default:
      // Handle error or invalid USART number
      return false;
  }

  tal_enable_clock(tx_pin);
  tal_enable_clock(rx_pin);
  if (ck_pin != 0) {
    tal_enable_clock(ck_pin);
    tal_set_mode(ck_pin, 2);
  }


  // Set alternate-function mode
  tal_set_mode(tx_pin, 2);
  tal_set_mode(rx_pin, 2);

  bool test_set_alt = set_alternate_function(channel, tx_pin, rx_pin, ck_pin);
  if (!test_set_alt) {
    return false;
  }


  // Ensure the clock pin is disabled for asynchronous mode
  // TODO: check on this
  if (!IS_USART_CHANNEL(channel)) {
    CLR_FIELD(UARTx_CR2[channel], UARTx_CR2_CLKEN);
  }

  // TODO: maybe calculate via using ints for mantissa/exponent field?
  uint32_t brr_value = clk_freq / baud_rate;
  if (IS_USART_CHANNEL(channel)) {
  WRITE_FIELD(USARTx_BRR[channel], USARTx_BRR_BRR_4_15, brr_value);

  // Set parity
  switch (parity) {
    case UART_PARITY_DISABLED:
      CLR_FIELD(USARTx_CR1[channel], USARTx_CR1_PCE);
      break;
    case UART_PARITY_EVEN:
      SET_FIELD(USARTx_CR1[channel], USARTx_CR1_PCE);
      CLR_FIELD(USARTx_CR1[channel], USARTx_CR1_PS);
      break;
    case UART_PARITY_ODD:
      SET_FIELD(USARTx_CR1[channel], USARTx_CR1_PCE);
      SET_FIELD(USARTx_CR1[channel], USARTx_CR1_PS);
      break;
  }


  // Set data length
  switch (data_length) {
    case UART_DATALENGTH_7:
      if (!parity) {
        // tal_raise(flag, "Invalid parity datasize combo");
        return false;
      }
      SET_FIELD(USARTx_CR1[channel], USARTx_CR1_Mx[0]);
      CLR_FIELD(USARTx_CR1[channel], USARTx_CR1_Mx[1]);
      break;
    case UART_DATALENGTH_8:
      CLR_FIELD(USARTx_CR1[channel], USARTx_CR1_Mx[0]);
      CLR_FIELD(USARTx_CR1[channel], USARTx_CR1_Mx[1]);
      break;
    case UART_DATALENGTH_9:
      if (parity) {
        // tal_raise(flag, "Invalid parity datasize combo");
        return false;
      }
      SET_FIELD(USARTx_CR1[channel], USARTx_CR1_Mx[0]);
      SET_FIELD(USARTx_CR1[channel], USARTx_CR1_Mx[1]);
      break;
  }


  // Enable FIFOs
  SET_FIELD(USARTx_CR1[channel], USARTx_CR1_FIFOEN);
} else {
  WRITE_FIELD(UARTx_BRR[channel], UARTx_BRR_BRR_4_15, brr_value);

  // Set parity
  switch (parity) {
    case UART_PARITY_DISABLED:
      CLR_FIELD(UARTx_CR1[channel], UARTx_CR1_PCE);
      break;
    case UART_PARITY_EVEN:
      SET_FIELD(UARTx_CR1[channel], UARTx_CR1_PCE);
      CLR_FIELD(UARTx_CR1[channel], UARTx_CR1_PS);
      break;
    case UART_PARITY_ODD:
      SET_FIELD(UARTx_CR1[channel], UARTx_CR1_PCE);
      SET_FIELD(UARTx_CR1[channel], UARTx_CR1_PS);
      break;
  }


  // Set data length
  switch (data_length) {
    case UART_DATALENGTH_7:
      if (!parity) {
        // tal_raise(flag, "Invalid parity datasize combo");
        return false;
      }
      SET_FIELD(UARTx_CR1[channel], UARTx_CR1_Mx[0]);
      CLR_FIELD(UARTx_CR1[channel], UARTx_CR1_Mx[1]);
      break;
    case UART_DATALENGTH_8:
      CLR_FIELD(UARTx_CR1[channel], UARTx_CR1_Mx[0]);
      CLR_FIELD(UARTx_CR1[channel], UARTx_CR1_Mx[1]);
      break;
    case UART_DATALENGTH_9:
      if (parity) {
        // tal_raise(flag, "Invalid parity datasize combo");
        return false;
      }
      SET_FIELD(UARTx_CR1[channel], UARTx_CR1_Mx[0]);
      SET_FIELD(UARTx_CR1[channel], UARTx_CR1_Mx[1]);
      break;
  }


  // Enable FIFOs
  SET_FIELD(UARTx_CR1[channel], UARTx_CR1_FIFOEN);

}

  dma_config_t dma_tx_stream = {
      .instance = tx_stream->instance,
      .stream = tx_stream->stream,
      .request_id = uart_dmamux_req[channel][1],
      .direction = tx_stream->direction,
      .src_data_size = tx_stream->src_data_size,
      .dest_data_size = tx_stream->dest_data_size,
      .priority = tx_stream->priority,
      .fifo_enabled = false, // FIFO disabled for tx
      .fifo_threshold = tx_stream->fifo_threshold,
      .callback = *callback, // We need to know if it failed.
  };
  dma_configure_stream(&dma_tx_stream);

  dma_config_t dma_rx_stream = {
      .instance = rx_stream->instance,
      .stream = rx_stream->stream,
      .request_id = uart_dmamux_req[channel][0],
      .direction = rx_stream->direction,
      .src_data_size = rx_stream->src_data_size,
      .dest_data_size = rx_stream->dest_data_size,
      .priority = rx_stream->priority,
      .fifo_enabled = false, // FIFO disabled for tx
      .fifo_threshold = rx_stream->fifo_threshold,
      .callback = *callback, // We need to know if it failed.
  };
  dma_configure_stream(&dma_rx_stream);

  // Save stream info
  dma_periph_streaminfo_t info = {.rx_instance = rx_stream->instance,
                                  .tx_instance = tx_stream->instance,
                                  .rx_stream = rx_stream->stream,
                                  .tx_stream = tx_stream->stream};
  uart_to_dma[channel] = info;

  // Enable the peripheral

  if (IS_USART_CHANNEL(channel)) {
    SET_FIELD(USARTx_CR1[channel], USARTx_CR1_TE);
    SET_FIELD(USARTx_CR1[channel], USARTx_CR1_RE);
    SET_FIELD(USARTx_CR1[channel], USARTx_CR1_UE);
  } else {
     SET_FIELD(UARTx_CR1[channel], UARTx_CR1_TE);
    SET_FIELD(UARTx_CR1[channel], UARTx_CR1_RE);
    SET_FIELD(UARTx_CR1[channel], UARTx_CR1_UE);
  }

  return true;
}

bool uart_write_async(uart_channel_t channel, uint8_t *tx_buff, uint32_t size) {
  // Verify parameters
  bool test_params = verify_transfer_parameters(channel, tx_buff, size);
  if (!test_params) {
    return false;
  }

  // Check if usart channel is busy
  if (uart_busy[channel]) {
    // tal_raise(flag, "USART channel is busy");
    return false;
  }
  uart_busy[channel] = true;

  // Configure DMA stream
  uart_context_t context = {
      .busy = &uart_busy[channel],
      .channel = channel,
  };
  uart_contexts[channel] = context;
  dma_transfer_t tx_transfer = {
      .instance = uart_to_dma[channel].tx_instance,
      .stream = uart_to_dma[channel].tx_stream,
      .src = tx_buff,
      .dest = (void *)UARTx_TDR[channel], // maybe revisit the cast... in dma transfer struct
      .size = size,
      .context = &uart_contexts[channel],
      .disable_mem_inc = false,
  };
  dma_start_transfer(&tx_transfer);

  // Enable the dma requests
  SET_FIELD(UARTx_CR3[channel], UARTx_CR3_DMAT);

  return true;
}

bool uart_read_async(uart_channel_t channel, uint8_t *rx_buff, uint32_t size) {
  // Verify parameters
  bool test_params = verify_transfer_parameters(channel, rx_buff, size);
  if (!test_params) {
    return false;
  }

  // Check if usart channel is busy
  if (uart_busy[channel]) {
    // tal_raise(flag, "USART channel is busy");
    return false;
  }
  uart_busy[channel] = true;

  // Configure DMA stream
  uart_context_t context = {
      .busy = &uart_busy[channel],
      .channel = channel,
  };
  uart_contexts[channel] = context;
  dma_transfer_t tx_transfer = {
      .instance = uart_to_dma[channel].tx_instance,
      .stream = uart_to_dma[channel].tx_stream,
      .src = (void *) UARTx_RDR[channel],
      .dest = rx_buff,
      .size = size,
      .context = &uart_contexts[channel],
      .disable_mem_inc = false,
  };
  dma_start_transfer(&tx_transfer);

  // Enable the dma requests
  SET_FIELD(UARTx_CR3[channel], UARTx_CR3_DMAT);
  return true;
}

bool uart_write_blocking(uart_channel_t channel, uint8_t *tx_buff,
                         uint32_t size) {
  // Verify parameters
  bool test_params = verify_transfer_parameters(channel, tx_buff, size);

  if (!test_params) {
    return false;
  }

  // Check if usart channel is busy
  // while (!READ_FIELD(USARTx_ISR[channel], USARTx_ISR_BUSY)) {
  //   asm("nop");
  //   // tal_raise(flag, "USART channel is busy");
  // }
  // uart_busy[channel] = true;

  // Transmit the data byte by byte
  for (uint32_t i = 0; i < size; i++) {
    if (!uart_write_byte(channel, tx_buff[i])) {
      // tal_raise(flag, "USART write timeout");
      // uart_busy[channel] = false;
      return false;
    }
  }

  // uart_busy[channel] = false;
  return true;
}

bool uart_read_blocking(uart_channel_t channel, uint8_t *rx_buff,
                        uint32_t size) {
  // Verify parameters
  bool test_params = verify_transfer_parameters(channel, rx_buff, size);
  if (!test_params) {
    return false;
  }

  // Check if usart channel is bus
  if(IS_USART_CHANNEL(channel)) {
  while (!READ_FIELD(USARTx_ISR[channel], USARTx_ISR_BUSY)) {
    asm("nop");
    // tal_raise(flag, "USART channel is busy");
  }
} else {
  while (!READ_FIELD(UARTx_ISR[channel], UARTx_ISR_BUSY)) {
    asm("nop");
    // tal_raise(flag, "USART channel is busy");
  }
}
  // uart_busy[channel] = true;


  // Receive the data byte by byte
  for (uint32_t i = 0; i < size; i++) {
    if (!uart_read_byte(channel, rx_buff+i)) {
      // tal_raise(flag, "USART read timeout");
      // uart_busy[channel] = false;
      return false;
    }
  }

  // uart_busy[channel] = false;
  return true;
}