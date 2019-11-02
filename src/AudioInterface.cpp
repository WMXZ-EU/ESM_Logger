/*
 * WMXZ Teensy audio library
 * Copyright (c) 2017 Walter Zimmer.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// AudioInterface.cpp
// interfaces to Audio streaming
//

// general teensy includes
#include "core_pins.h"
#include "usb_serial.h"

#include "AudioInterface.h"

uint16_t c_buff::put(uint32_t * data, uint16_t len)
{  // consider overrun if nw is len bytes above nr + mbuf
  if( (nw+len) > (nr+mbuf)) return 0;
  //
  for(int ii=0; ii<len; ii++) buffer[(nw+ii) % mbuf] = data[ii];
  nw += len;
  return len;
}

//
int16_t *c_buff::get(uint16_t len)
{ if(nr+len > nw) return 0;

  int16_t *ptr = (int16_t *) &buffer[nr % mbuf];
  nr += len;
  return ptr;
}

/************************ AudioInterface **************************************************************/
//

void AudioInterface::init(c_buff *store, int fsamp)
{ 	audioStore = store;
	jfs1=fsamp;
	jfs2=44100;

	// remove tailing zeros
	while( ((jfs1 % 10) ==0) && ((jfs2 % 10))==0) { jfs1 /= 10; jfs2 /= 10; } 
	jfs1 *= 10; jfs2 *= 10;
	//
	n_src=(uint16_t)((AUDIO_BLOCK_SAMPLES*jfs1)/jfs2); // number od samples in src_buffer
	//
	sc = (float) (n_src) / (float) (AUDIO_BLOCK_SAMPLES);	
}

void AudioInterface::interpolate(int16_t *dst, const int16_t *src)
{

	for(int ii=0; ii<AUDIO_BLOCK_SAMPLES; ii++) dst[ii]=0;
	for(int ii=0; ii<AUDIO_BLOCK_SAMPLES; ii++)
	{	
		uint32_t j0;
		float tx;
		
		tx = ii*sc;
		// index into original
		j0 = (uint32_t)tx;
		// handle boundary
		if(j0 == 0) j0 =1;
		if(j0 == (n_src-1)) j0=n_src-2;
		
		// distance to index 
		dx1 = 0*(tx - (float)j0);
		dst[ii] = src[2*j0]
					+ ((src[2*j0+2] - src[2*j0-2])/2)*dx1
					+ (((src[2*j0+2] - src[2*j0]) - (src[2*j0] - src[2*j0-2]))/4)*dx1*dx1;
	}
}

void AudioInterface::update(void)
{	const int16_t *src;
	int16_t *dst;
	audio_block_t *left, *right;
	int16_t *src_buffer;
	//
	src_buffer = audioStore->get(n_src);
	if(!src_buffer) { return; }
	
	left = allocate();  if (!left) return;
	right = allocate(); if (!right) return;
	
	src = &src_buffer[0];
	dst = left->data;
	//
	interpolate(dst, src);
	
	src = &src_buffer[1];
	dst = right->data;
	//
	interpolate(dst, src);
	
	transmit(left,0);
	transmit(right,1);
	release(left);
	release(right);
}
