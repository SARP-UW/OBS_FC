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
 * @file peripheral/gpio.h
 * @authors Joshua Beard
 * @brief Driver implementation for the GPIO
 */

#include "internal/mmio.h"
#include "gpio.h"


/**
 * port_index_from_pin[overall pin #] = 100 * port + PIN # Within Port
 * -1 if pin doesn't exist
*/
int32_t port_index_from_pin[140] = {-1,402,403,404,405,406,-1,-1,-1,213,
                                    214,215,-1,-1,-1,-1,-1,-1,-1,-1,
                                    506,507,508,509,510,700,701,-1,200,201,
                                    -1,-1,-1,-1,-1,-1,-1,0,1,2,
                                    3,-1,-1,4,5,6,7,204,205,100,
                                    101,102,511,514,515,-1,-1,407,408,409,
                                    410,411,412,413,414,415,110,111,-1,-1,
                                    -1,-1,112,113,114,115,308,309,310,-1,
                                    -1,311,312,313,314,315,606,607,608,-1,
                                    -1,-1,-1,206,207,208,209,8,9,10,
                                    11,12,13,-1,-1,-1,-1,14,15,210,
                                    211,212,300,301,302,303,304,305,-1,-1,
                                    306,307,609,610,611,612,613,614,-1,-1,
                                    103,104,105,106,107,-1,108,109,400,401};

                              
typedef enum {
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_I,
    GPIO_PORT_J,
    GPIO_PORT_K,
} gpio_port_t;


void tal_set_mode(int pin, int mode)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ // This means this pin number isn't on the board
    return; 
  }
  int port = v / 100;
  int index = v - 100 * port;

  WRITE_FIELD(GPIOx_MODER[port], GPIOx_MODER_MODEx[index], mode);
}

void tal_set_drain(int pin, int drain)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ // This means this pin number isn't on the board
    return; 
  }
  int port = v / 100;
  int index = v - 100 * port;

  WRITE_FIELD(GPIOx_OTYPER[port], GPIOx_OTYPER_OTx[index], drain);
}

void tal_set_speed(int pin, int speed)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ // This means this pin number isn't on the board
    return; 
  }
  int port = v / 100;
  int index = v - 100 * port;

  WRITE_FIELD(GPIOx_OSPEEDR[port], GPIOx_OSPEEDR_OSPEEDx[index], speed);
}


void tal_pull_pin(int pin, int pull)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ 
    return; 
  }
  int port = v / 100;
  int index = v - 100 * port;

  switch (pull)
  {
    case 1:{
      WRITE_FIELD(GPIOx_PUPDR[port], GPIOx_PUPDR_PUPDx[index], 1);
      break;
    }
    case 0:{
      WRITE_FIELD(GPIOx_PUPDR[port], GPIOx_PUPDR_PUPDx[index], 0);
      break;
    }
    case -1:{
      WRITE_FIELD(GPIOx_PUPDR[port], GPIOx_PUPDR_PUPDx[index], 2);
      break;
    }
    
    default:{
      break;
    }
  }
}

void tal_set_pin(int pin, int value)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ 
    return; 
  }
  int port = v / 100;
  int index = v - 100 * port;

  switch (value){
    case 0:{
      WRITE_FIELD(GPIOx_ODR[port], GPIOx_ODR_ODx[index], 0);
      break;
    }
    case 1:{
      WRITE_FIELD(GPIOx_ODR[port], GPIOx_ODR_ODx[index], 1);
      break;
    }

    default:{
      break;
    }
  }
}

void tal_alternate_mode(int pin, int value)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ 
    return; 
  }
  int port = v / 100;
  int index = v - 100 * port;

  if(index <= 7){
    // use AFRL
    WRITE_FIELD(GPIOx_AFRL[port], GPIOx_AFRL_AFSELx[index], value);

  }else{
    // use AFRH
    WRITE_FIELD(GPIOx_AFRH[port], GPIOx_AFRH_AFSELx[index - 8], value); // subtract 8 from index for offset
  }
}

bool tal_read_pin(int pin)
{
  int v = port_index_from_pin[pin];
  if(v == -1){ 
    return false;  // throw error
  }
  int port = v / 100;
  int index = v - 100 * port;
  
  uint32_t read_val = READ_FIELD(GPIOx_IDR[port], GPIOx_IDR_IDx[index]);
  return read_val == 1;
}

bool tal_enable_clock(int pin) {
    gpio_port_t port = port_index_from_pin[pin] / 100;
    switch (port) {
        case (GPIO_PORT_A): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
            return true;
        case (GPIO_PORT_B): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            return true;
        case (GPIO_PORT_C): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOCEN);
            return true;
        case (GPIO_PORT_D): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIODEN);
            return true;
        case (GPIO_PORT_E): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOEEN);
            return true;
        case (GPIO_PORT_F): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOFEN);
            return true;
        case (GPIO_PORT_G): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOGEN);
            return true;
        case (GPIO_PORT_H): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOHEN);
            return true;
        case (GPIO_PORT_I): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOIEN);
            return true;
        case (GPIO_PORT_J): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOJEN);
            return true;
        case (GPIO_PORT_K): // 
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOKEN);
            return true;
    }
    return false;
}

bool tal_disable_clock(int pin) {
    gpio_port_t port = port_index_from_pin[pin] / 100;
    switch (port) {
        case (GPIO_PORT_A): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
            return true;
        case (GPIO_PORT_B): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            return true;
        case (GPIO_PORT_C): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOCEN);
            return true;
        case (GPIO_PORT_D): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIODEN);
            return true;
        case (GPIO_PORT_E): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOEEN);
            return true;
        case (GPIO_PORT_F): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOFEN);
            return true;
        case (GPIO_PORT_G): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOGEN);
            return true;
        case (GPIO_PORT_H): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOHEN);
            return true;
        case (GPIO_PORT_I): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOIEN);
            return true;
        case (GPIO_PORT_J): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOJEN);
            return true;
        case (GPIO_PORT_K): // 
            CLR_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOKEN);
            return true;
    }
    return false;
}
