//Copyright 2017 by Walter Zimmer
// Version 18-05-17
//
// general teensy includes
#include <kinetis.h>
#include <core_pins.h>
#include "usb_serial.h"
//
// general (WMXZ) core library
#include "dma.h"
#include "I2S.h"
//
// local class definition
#include "ICS43432.h"

/**
 *  connections
 *  defined in I2S.c			  wmxz
 *  pin 11, PTC6, I2S0_RX_BCLK    blue
 *  pin 12, PTC7, I2S0_RX_FS      green
 *  pin 13, PTC5, I2S0_RXD0       yellow
 *  pin 30, PTC11,I2S0_RXD1 T3.2
 *  pin 38, PTC11,I2S0_RXD1 T3.6
 *  GND                           white
 *  3V3                           red
 *  L/R                           purple
 */

  extern int iscl[];

uint32_t c_ICS43432::init(int32_t fsamp, int32_t *buffer, uint32_t nbuf, uint16_t nch)
{
  i2s_init();
  
  float fs = i2s_speedConfig(ICS43432_DEV,N_BITS, fsamp);
  if(fs<1.0f) return 0;

  if(nch>2)  
  	i2s_config(1, N_BITS, I2S_RX_2CH, 0); // both RX channels
  else
	  i2s_config(1, N_BITS, 0, 0);  // only 1 RX channel
  i2s_configurePorts(2);

  DMA_init();
  i2s_setupInput(buffer,nbuf,2,5); //port, prio (8=normal)
  return (uint32_t) fs;
}

void c_ICS43432::start(void)
{
  i2s_enableInputDMA();
  DMA_startAll();
  i2s_startInput();
}

void c_ICS43432::stop(void)
{
  i2s_stopInput();
  DMA_haltAll();
}

void c_ICS43432::exit(void) { i2s_stopClock();}
