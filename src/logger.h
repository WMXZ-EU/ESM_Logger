/* wmxzAudio Library for Teensy 3.X
 * Copyright (c) 2017, Walter Zimmer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef LOGGER_H
#define LOGGER_H

typedef struct
{
  uint32_t rtc;
  uint32_t t0;
  uint32_t nch;
  uint32_t fsamp;
  uint32_t fsize;
  uint32_t nsamp;
  uint32_t hsize;
  uint32_t nclst;
  uint32_t fill[128-8];
} header_s;

/*
 * simple data pool (no lock-release)
 * T type of data 
 * nb number of data blocks
 * nd size of data block
 */
template <typename T, int nb, int nd>
class store
{ T pool[nb*nd];

  public:
  store(void) {;}
  T *fetch(int ii) {return &pool[ii*nd];}
  void release(uint16_t ii) {;}
};


#include "mfs.h"
c_mFS mFS;
header_s header;

/*--------------  - uSDLogger class          ------------------*/
class uSD_IF
{
  public:
  uSD_IF(void) {;}
  void init(void);
  void reset(void) {fileStatus=0;}
  int32_t save(char *fmt, int mxfn, int max_mb);
  int32_t save(int max_mb);
  uint32_t overrun=0;
  uint32_t maxBlockSize=0;
  int16_t isRunning = 0; // tell upper classes 

  private:
  virtual void *drain(void) =0;
  virtual int16_t write(void *src) =0;
  virtual void haveFinished(void)=0;
  uint16_t fileStatus = 0;
  uint32_t ifn = 0;
  uint32_t loggerCount = 0;
  //
};

/*------- AudioRecorderLogger class and short methods -----------*/
/*
 * inplements Buffered Logger
 * T type of data
 * nq number of data blocks to buffer
 * nd size of data block
 * na number of data blocks in write buffer
 */
template <typename T, int nq, int nd, int na>
class Logger : public uSD_IF
{
public:
  Logger (void) : head(0), tail(0), enabled(0)
  { maxBlockSize = na*nd*sizeof(T);}

  void start(void) { clear(); reset(); isRunning=1; enabled = 1; }
  void stop(void) { isRunning=0; } // tell uSD_IF
  void stopnow(void) { isRunning=-1; } // tell uSD_IF
  //
  void clear(void);
  //
  void *drain(void);
  int16_t write(void *src);
  void haveFinished(void) {enabled=0;} // got signal from uSD_IF

private:
  store<T,nq,nd> pool;
  T* queue[nq];
  int16_t head, tail, enabled;

  T buffer[na*nd]; // for draining data
};

/*--------------- larger AudioRecorderLogger methods ------------------*/
template <typename T, int nq, int nd, int na>
void Logger<T,nq,nd,na>:: clear(void)
  {
    uint16_t t = tail;
    while (t != head) {
      if (++t >= nq) t = 0;
      pool.release(t);
      queue[t]=0; // remove address from queue
    }
    tail = t;
  }

template <typename T, int nq, int nd, int na>
int16_t Logger<T,nq,nd,na>:: write(void *inp)
  {
    int16_t h;

    if(!enabled) return 0; // don't do anything
    
    h = head + 1;
    if (h >= nq) h = 0;
    if (h == tail) {  // disaster
      overrun++;
      // simply ignore new data
      return -1;
    } 
    else 
    { queue[h] = pool.fetch(h);
      T *ptr = queue[h];
      if(ptr)
      { T *src = (T*) inp;
        for(int ii=0; ii<nd; ii++) ptr[ii]=src[ii];
        head = h;
        return head;
      }
      else
        return -1;
    }
  }
  
template <typename T, int nq, int nd, int na>
void * Logger<T,nq,nd,na>:: drain(void)
  {
    uint16_t n;
    if(head>tail) n=head-tail; else n = nq + head -tail;

    int16_t nb=na; // na is number of buffers to fetched from queue and written to disk
    if(n>nb)
    {
      T *bptr = buffer;
      //
      uint16_t t = tail;
      while(--nb>=0)
      {
        if (t != head) 
        {
          if (++t >= nq) t = 0;
          
          // copy to buffer     
          { T *src = queue[t];
            if(src)
            { for(int jj=0; jj<nd; jj++) bptr[jj]=src[jj];
              pool.release(t);
              queue[t]=0;
            }
          }
          tail = t;
          bptr += nd;
        }
      }
      return (void *)buffer;
    }
    return 0;
  }

/*
 * ************** uSD_logger methods *****************
 * 
 */
void uSD_IF::init(void)
{
  mFS.init();
  fileStatus=0;
}

#include <time.h>
struct tm seconds2tm(uint32_t tt);

uint16_t generateFilename(char *dev, char *filename)
{
  struct tm tx=seconds2tm(RTC_TSR);;
  sprintf(filename,"%s_%04d%02d%02d_%02d%02d%02d.bin",dev,
          tx.tm_year, tx.tm_mon, tx.tm_mday,
          tx.tm_hour, tx.tm_min, tx.tm_sec);
  return 1;
}

int32_t uSD_IF::save(int max_mb )
{ // does also open/close a file when required
  //
  static uint16_t isLogging = 0; // flag to ensure single access to function

  char filename[80];
  uint16_t nbuf = maxBlockSize;
  uint32_t maxLoggerCount = (max_mb*1024*1024)/maxBlockSize;

  if (isLogging) return 1; // we are already busy (should not happen)
  isLogging = 1;

  if(fileStatus==0)
  { // open new file
    if(!generateFilename((char *)parameters.name,filename))  // have end of acquisition reached, so end operation
    { fileStatus = 4;
      isLogging = 0; return 0; // tell calling loop() we have error
    } // end of all operations

    mFS.open(filename);
    #if DO_DEBUG > 0
        Serial.printf(" %s\n\r",filename);
        Serial.printf(" %d blocks max: %d  MB\n\r",maxLoggerCount,max_mb);
    #endif
    loggerCount=0;  // count successful transfers
    overrun=0;      // count buffer overruns
    //
    if (!mFS.write((uint8_t*)&header, sizeof(header_s)))
      fileStatus = 3; // close file on write failure
    else
      fileStatus = 2; // flag as open
  }

  if(fileStatus==2)
  { 
    // write to file
    uint8_t *buffer=(uint8_t*)drain();
    if(buffer)
    {
      if (!mFS.write(buffer, nbuf)){ fileStatus = 3;} // close file on write failure
      if(fileStatus == 2)
      { loggerCount++;
        if(loggerCount == maxLoggerCount)
        { fileStatus= 3;}
        #if DO_DEBUG == 2
          else
          { if (!(loggerCount % 10)) Serial.printf(".");
            if (!(loggerCount % 640)) {Serial.println(); }
            Serial.flush();
          }
        #endif
      }
    }
    if(isRunning<0) 
    { fileStatus=3; // flag to stop logging
      isRunning=0;  // tell close to finish to finish aquisition
    }
  }

  if(fileStatus==3)
  {
    //close file
    mFS.close();
    #if DO_DEBUG ==2
        Serial.printf("\n\r overrun: (%d)\n\r",overrun);
    #endif
    //
    fileStatus= 0; // flag file as closed   
  }

  if(isRunning==0) // we should stop logging
  { haveFinished(); fileStatus = 4; 
  }

  isLogging = 0;
  if(fileStatus==4) { return -1; } // don't do anything anymore 
  return 1; 
}

int32_t uSD_IF::save(char *fmt, int mxfn, int max_mb )
{ // does also open/close a file when required
  //
  static uint16_t isLogging = 0; // flag to ensure single access to function

  char filename[80];

  if (isLogging) return 0; // we are already busy (should not happen)
  isLogging = 1;

  if(fileStatus==4) { isLogging = 0; return 1; } // don't do anything anymore

  if(fileStatus==0)
  {
    // open new file
    ifn++;
    if (ifn > (unsigned)mxfn) // have end of acquisition reached, so end operation
    { fileStatus = 4;
      isLogging = 0; return -1; // tell calling loop() to stop ACQ
    } // end of all operations

    sprintf(filename, fmt, (unsigned int)ifn);
    mFS.open(filename);
#if DO_DEBUG >0
    Serial.printf(" %s\n\r",filename);
#endif    
    loggerCount=0;  // count successful transfers
    overrun=0;      // count buffer overruns
    //
    fileStatus = 2; // flag as open
    isLogging = 0; return 1;
  }

  if(fileStatus==2)
  { 
    // write to file
    uint16_t nbuf = maxBlockSize;
    uint32_t maxLoggerCount = (max_mb*1024*1024)/maxBlockSize;
    uint8_t *buffer=(uint8_t*)drain();
    if(buffer)
    {
      if (!mFS.write(buffer, nbuf))
      { fileStatus = 3;} // close file on write failure
      if(fileStatus == 2)
      {
        loggerCount++;
        if(loggerCount == maxLoggerCount)
        { fileStatus= 3;}
#if DO_DEBUG == 2
        else
        {
          if (!(loggerCount % 10)) Serial.printf(".");
          if (!(loggerCount % 640)) {Serial.println(); }
          Serial.flush();
        }
#endif
      }
    }
    if(fileStatus==2){ isLogging = 0; return 1; }
  }

  if(fileStatus==3)
  {
    //close file
    mFS.close();
#if DO_DEBUG ==2
    Serial.printf("\n\r(%d)\n\r",overrun);
#endif    
    //
    fileStatus= 0; // flag file as closed   
    isLogging = 0; return 1;
  }
  
  isLogging=0; return 0;
}

#if USE_LUX==1
// uncomment all LightSensor lines if sensor is attached
#include <Wire.h>
#include <BH1750FVI.h>

BH1750FVI LightSensor;

int getLightSensor()
{
  int lux=0;
    LightSensor.begin();
    LightSensor.SetAddress(Device_Address_H);//Address 0x5C
    LightSensor.SetMode(Continuous_H_resolution_Mode);
    lux = LightSensor.GetLightIntensity();
  return lux;
}

void logLightSensor(void)
{
  int lux =0;
  lux=getLightSensor();
  
  struct tm tx=seconds2tm(RTC_TSR);  
  
  char txt[80];
  sprintf(txt,"%4d/%02d/%02d %02d:%02d %d\r\n", 
      tx.tm_year, tx.tm_mon, tx.tm_mday,tx.tm_hour, tx.tm_min, lux);
  mFS.logText((char *)"lux.txt",(char *)txt);
}
#endif

void storeConfig(void * ptr)
{ char text[32];
  uint16_t *data = (uint16_t *) ptr;
  mFS.open((char*)"Config.txt", O_CREAT|O_WRITE|O_TRUNC);
  for(int ii=0; ii<6; ii++)
    {sprintf(text,"%2d\r\n",data[ii]); mFS.write((uint8_t*)text,strlen(text));}
  mFS.write((uint8_t *)&data[6],4);
  mFS.close();
  
}

void readConfig(void * ptr)
{
  char text[32];
  uint16_t *data = (uint16_t *) ptr;
  if(!mFS.open((char*)"Config.txt",O_RDONLY)) return;
  for(int ii=0; ii<6; ii++)
    {mFS.read((uint8_t*)text,4); sscanf(text,"%d",&data[ii]);}
  mFS.read((uint8_t*)&data[6],4);
  mFS.close();
}

#endif

