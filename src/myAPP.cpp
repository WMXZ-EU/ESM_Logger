// Copyright 2017 by Walter Zimmer
// Version V1g 06-10-17
//
// in addition to teensy core
// this application needs to be set for 
//  Serial (logger only)
//  Audio  (usb-audio only)
//
// this application needs support from
// SDFS         // for logger (Bill Greiman's latest library)
//
//-------------------- Connections ------------------------
// 18-sep-2017: 
// for ICS4343x microphones
// T3.2a  T3.6a             Mic1  Mic2  Mic3  Mic4   
// GND    GND               GND   GND   GND   GND    
// 3.3V   3.3V              VCC   VCC   VCC   VCC    
// Pin11  Pin11     BCLK    CLK   CLK   CLK   CLK 
// Pin12  Pin12     FS      WS    WS    WS    WS  
// Pin13  Pin13     RXD0    SD    SD    --    --  
// Pin30  Pin38     RXD1    --    --    SD    SD  
// GND    GND               L/R   --    L/R   --
// 3.3    3.3               --    L/R   --    L/R
//
// T3.xa are alternative pin settings for pure RX mode (input only)

// PJRC Audio uses for I2S MCLK and therefore uses RX sync'ed internally to TX
// that is, TX_BCLK and TX_FS are sending out data but receive data on RDX0, RDX1
// the modified (local) implementation does not use MCLK and therefore can use
// RX_BCLK and RX_FS
//
// PJRC pin selection
// Pin11 MCLK
// Pin9  TX_BCLK
// Pin23 TX_FS
//
// RX_ONLY pin selection
// Pin11  RX_BCLK
// Pin12  RX_FS

// general teensy includes
#include "core_pins.h"
#include "usb_serial.h"
//
// application specifific includes
#include "myApp.h"

#define SERIALX Serial // needed for remote configuration could be Serial1 if use of HW serial

//default parameters
// times for hibernate 
#define ON_TIME  1 // acq on time  // set to zero when no periodic hibernate (actual file will be finished)
#define OFF_TIME 9 // acq off time 
// daily activity window
#define T1 21  // first hour of time slots 
#define T2 1 // second hour of time slots 
#define T3 4 // third hour of time slots 
#define T4 6 // last hour of time slots  


typedef struct
{
  uint16_t on_time;
  uint16_t off_time;
  uint16_t first_hour;
  uint16_t second_hour;
  uint16_t third_hour;
  uint16_t last_hour;
  char name[5];
} parameters_s;
parameters_s parameters={ON_TIME,OFF_TIME,T1,T2,T3,T4,"WMXZ"};
uint16_t par_mods=0;

#define DO_DEBUG 2 

#if ON_TIME >0
  #undef DO_DEBUG
  #define DO_DEBUG 0
#endif

// enable either logger
// or USB_AUDIO
#define DO_LOGGER

#ifndef DO_LOGGER
  #define DO_USB_AUDIO
#endif

// some definitions
#define F_SAMP 44100 // tested with F_CPU=180MHz
#define N_CHAN 1   // number of channels can be 1, 2, 4 // effects only logging


#ifdef DO_USB_AUDIO
  #define AUDIO_SHIFT 4 // shift to right (or attenuation)
  #define ICHAN_LEFT  0 // index for left usb_audio channel
  #define ICHAN_RIGHT 0 // index for right usb_audio channel
#endif

#ifdef DO_LOGGER
  // for AudioRecordLogger
  #define NQ  (300/N_CHAN) // number of elements in queue
  // NCH*NQ should be <300 (for about 200 kB RAM usage and 32 bit words) 
  
  // for uSD_Logger
  #define MAX_MB 40   // max (expected) file size in MB // is also file size if ON_TIME==0
#endif
/***********************************************************************/
#include "ICS43432.h" // defines also N_BITS

// Note: 
// change either F_CPU or F_SAMP if I2S setup fails to configure
// i.e. there are no DMA interrupts and 'i2sInProcessing' is not running
// typically this happens if the clock generation has a too high multiplier
// e.g. F_CPU=168 MHz and F_SAMP =  96000 fails to run (cannot find proper dividers)
//  but F_CPU=168 MHz and F_SAMP = 100000 runs fine
//

/********************** I2S parameters *******************************/
c_ICS43432 ICS43432;
#define MSB_CORRECTION

extern "C" void i2sInProcessing(void * s, void * d);

#define N_SAMP 128
#if N_CHAN== 1
  #define ICH 0 // is 0 for first channel
#endif

#if N_CHAN<=2
  #define I2S_CHAN 2
#else
  #define I2S_CHAN 4
#endif

#if N_BITS == 32
  typedef int32_t DATA_T;
#else
  typedef int16_t DATA_T;
#endif

#define N_BUF (2 * I2S_CHAN * N_SAMP)    // dual buffer size for DMA 

DATA_T i2s_rx_buffer[N_BUF];          // buffer for DMA


//------------------------ Asynchronous Blink ------------------------------
void blink(uint32_t msec)
{ static uint32_t to=0;
  uint32_t t1 = millis();
  if(t1-to<msec) {yield(); return;}
  digitalWriteFast(13,!digitalReadFast(13)); 
  to=t1;
}
/*
 * *************** Acquisition interface ***********************************
 */

uint32_t i2sProcCount=0;
uint32_t i2sBusyCount=0;
uint32_t i2sWriteErrorCount=0;

inline void mCopy(int32_t *dst, int32_t *src, uint32_t len)
{ for(uint32_t ii=0;ii<len;ii++) dst[ii]=src[ii];
}

inline uint16_t acqSetup(void)
{
  // initialize and start ICS43432 interface
  uint32_t fs = ICS43432.init(F_SAMP, i2s_rx_buffer, N_BUF, I2S_CHAN);
  if(fs>0)
  {
    #if DO_DEBUG>0
      Serial.printf("Fsamp requested: %.3f kHz  got %.3f kHz\n\r" ,
          F_SAMP/1000.0f, fs/1000.0f);
      Serial.flush();
    #endif
    return 1;
  }
  return 0;
}

inline void acqStart(void)
{ 
  #if DO_DEBUG == 2
    Serial.println("Start Acq");
  #endif
  ICS43432.start();
}
inline void acqStop(void)
{ 
  #if DO_DEBUG == 2
    Serial.println("Stop Acq");
  #endif
  ICS43432.stop();
}

inline void acqExit(void)
{
  ICS43432.exit();
}

inline void acqLoop(void)
{
  static uint32_t t0=0;
  static uint32_t loopCount=0;

  uint32_t t1=millis();
  if (t1-t0>1000) // log to serial every second
  { static uint32_t icount=0;
    #if DO_DEBUG>0
      Serial.printf("%4d %d %d %d %d %d %.3f kHz\n\r",
            icount, loopCount, i2sProcCount,i2sBusyCount, i2sWriteErrorCount, 
            N_SAMP,((float)N_SAMP*(float)i2sProcCount/1000.0f));
    #endif
    i2sProcCount=0;
    i2sBusyCount=0;
    i2sWriteErrorCount=0;
    loopCount=0;
    t0=t1;
    icount++;
  }
  loopCount++;
}


/*******************Logger Interface*******************************************/
#ifdef DO_LOGGER
  #include "logger.h"
  // for uSD_Logger write buffer
  // the sequence (64,32,16) is for 16kB write buffer
  #if N_CHAN==1
    #define NAUD 64
  #elif N_CHAN==2
    #define NAUD 32
  #elif N_CHAN==4
    #define NAUD 16
  #endif
  Logger<DATA_T, NQ, N_CHAN*N_SAMP, NAUD>  logger; 

  DATA_T data1[N_SAMP];
  
#endif

//
/*******************USB-Audio Interface*******************************************/
#ifdef DO_USB_AUDIO
  #include "kinetis.h"
  #include "core_pins.h"
  //
  #include "AudioStream.h"
  #include "usb_audio.h"
  //
  class AudioIF : public AudioStream
  {
  public:
          AudioIF() : AudioStream(0, NULL){update_responsibility = update_setup(); }
          void trigger(void){ if (update_responsibility) AudioStream::update_all();}
          virtual void update(void) {;}
  private:
          static bool update_responsibility;
  };
  bool AudioIF::update_responsibility = false;
  
  AudioIF audio_if;
  
  #include "AudioInterface.h"   // contains function implementation

  #define N_DAT (128*(F_SAMP/100)/441)  // number of samples per received DMA interrupt  

  static uint32_t audioBuffer[3*N_DAT]; // 3 buffers for audio xfer (need at least 2)
  c_buff audioStore(audioBuffer,sizeof(audioBuffer)/4);
  
  AudioInterface  interface(&audioStore,F_SAMP);
  AudioOutputUSB  usb;
  AudioConnection patchCord1(interface,0,usb,0);
  AudioConnection patchCord2(interface,1,usb,1); 

  inline void mExtract(int16_t *dst, int32_t *src, uint32_t len);
  inline void usbAudio_write(int16_t *buf, uint32_t len);

  int16_t dst16[I2S_CHAN * N_SAMP];  // buffer for onward processing

#endif



/************************Process specific code ********************************/
void i2sInProcessing(void * s, void * d)
{
	static uint16_t is_I2S=0;

	i2sProcCount++;
	if(is_I2S) {i2sBusyCount++; return;}
	is_I2S=1;
 
	int32_t *src = (int32_t *) d;

	// for ICS43432 need first shift left to get correct MSB
	// shift 8bit to right to get data-LSB to bit 0
  #ifdef MSB_CORRECTION
  	for(int ii=0; ii<I2S_CHAN*N_SAMP;ii++) { src[ii]<<=1; src[ii]>>=8;}
  #endif

	#ifdef DO_LOGGER
    #if N_CHAN==1
      DATA_T *logData = data1; 
      for(int ii=0; ii< N_SAMP; ii++) logData[ii]=src[ICH+2*ii];
    #else
      DATA_T *logData = src;
    #endif
		if(logger.write(logData)==INF) //store always original data
    { // have write error
      i2sWriteErrorCount++;
    }
	#endif

	#ifdef DO_USB_AUDIO
    mExtract(dst16,src,ICS_CHAN*N_SAMP);
		usbAudio_write(dst16,N_SAMP);
	#endif

  is_I2S=0;
  // 
  #ifdef DO_USB_AUDIO
    audio_if.trigger(); 
  #endif
}


/*
 * ********************************************************************************
 */
#ifdef DO_USB_AUDIO
	static int16_t waveform[2*N_SAMP]; // store for stereo usb-audio data

	inline void usbAudio_init(void)
	{	AudioMemory(8);
	}

	inline void usbAudio_write(int16_t *buf, uint32_t len)
	{	// prepare data for USB-Audio
		// extract data from I2S buffer
		for(int ii=0; ii<len; ii++)
		{	waveform[2*ii]  =(buf[ICHAN_LEFT +ii*I2S_CHAN]);
			waveform[2*ii+1]=(buf[ICHAN_RIGHT+ii*I2S_CHAN]);
		}
		// put data onto audioStore
		audioStore.put((uint32_t *) waveform, N_SAMP); //  2x 16-bit channels
	}
 
   inline void mExtract(int16_t *dst, int32_t *src, uint32_t len)
  { for(uint32_t ii=0;ii<len;ii++) dst[ii]=(src[ii]>>AUDIO_SHIFT);
  }

#endif

#ifdef DO_LOGGER
  extern header_s header;
	void loggerSetup(uint32_t nch, uint32_t fsamp, uint32_t nsamp)
	{
		header.nch = nch;
		header.nsamp = nsamp;
		header.fsamp = fsamp;
    logger.init();
	}
 
  inline void loggerStart(void)
  { 
    #if DO_DEBUG>0
      Serial.println("Start Logger"); 
    #endif
    logger.start();
  }
  inline void loggerStop(int16_t flag)
  { 
    #if DO_DEBUG>0
      Serial.println("Stop Logger"); 
    #endif
    if(flag)
      logger.stopnow();
    else
      logger.stop();
  }

	inline uint16_t loggerLoop(void){  return logger.save(MAX_MB);	}
#endif

/*
 * ************************** Arduino compatible Setup********************************
 */
#include "hibernate.h"

void go_hibernate(uint32_t seconds)
{ digitalWriteFast(23,LOW); // turn sensor an micro OFF 
  for(int ii=0;ii<24;ii++) pinMode(ii,INPUT);   
  hibernate(seconds);
}

void check_hibernate( parameters_s *par, int flag);


int16_t doMenu();
int16_t parMods=0;
static void printAll(void);

int haveAcq=0;
uint32_t startTime=0;
int loopStatus=0; // 0: stopped, 1: stopping, 2: running
int doHibernate=0;
  

// to disable EventResponder
// (https://forum.pjrc.com/threads/46442-Minimal-Blink-fails-with-void-yield()?p=153602&viewfull=1#post153602)
extern "C" volatile uint32_t systick_millis_count;
void mySystick_isr(void){ systick_millis_count++;}
void yield(void){}
//
// Arduino Setup
void setup(void)
{
  // redirect Systick 
  _VectorsRam[15] = mySystick_isr;

  // check first (with pin2 set to low) if we wanted to enter menu mode
  pinMode(2,INPUT_PULLUP);
  delay(1000);
  pinMode(23, OUTPUT);
  digitalWriteFast(23,LOW); // turn sensor and mic ON 
  
  if(digitalReadFast(2)==LOW)
  { 
    // signal begin of menu
    uint32_t tt=millis();
    pinMode(13,OUTPUT); // for LED
    while(!SERIALX) blink(1000);
    digitalWrite(13,LOW); // switch of Led

    SERIALX.begin(9600);

    #ifdef DO_LOGGER
      loggerSetup(N_CHAN, F_SAMP, N_SAMP);
      readConfig(&parameters);
    #endif
    printAll();

    parMods=0;
    int16_t ret=0;
    do ret=doMenu(); while(ret==0);
    //expect -1 to continue with program and a value >0 to hibernate for (val) minutes
    
    // signal end of menu
    tt=millis();
    pinMode(13,OUTPUT); // for LED
    while(millis()-tt<1000) blink(100);
    digitalWrite(13,HIGH); // switch of Led

    if(parMods)
    {
      #ifdef DO_LOGGER
        loggerSetup(N_CHAN, F_SAMP, N_SAMP);
        storeConfig(&parameters);
      #endif
      printAll();
    }

    if(ret>0) go_hibernate(ret*60);
  }
  else
  {
    #ifdef DO_LOGGER
      loggerSetup(N_CHAN, F_SAMP, N_SAMP);
      readConfig(&parameters);
      // for debugging
      uint32_t tt=millis();
      pinMode(13,OUTPUT); // for LED
      while(millis()-tt<1100) blink(500);
    #endif
  }
  
	#ifdef DO_USB_AUDIO
		usbAudio_init();
	#endif

  // limit acquisition to specific hours of day
  check_hibernate(&parameters,0);
       
	#if DO_DEBUG>0
    // wait for serial line to come up
    pinMode(13,OUTPUT); // for LED
    pinMode(13,HIGH);
		while(!Serial) blink(500);
		Serial.println("ESM Logger and Monitor");
  #else
    // blink for 1 second
    while(millis()<1000) blink(100);
	#endif

	#ifdef DO_LOGGER
    logLightSensor();  
	#endif

	haveAcq=acqSetup();
 loopStatus=0;
 doHibernate=0;
 #if ON_TIME > 0
   acqStart();
   #ifdef DO_LOGGER
     delay(300); // delay logger to allow acq to settle down
     loggerStart(); 
   #endif
   startTime=millis();
 #endif

}

int menu(void);

/*************************** Arduino loop ************************************/
void loop(void)
{ 
  if(!haveAcq) return;
  //
  if(doHibernate && (loopStatus<2)) 
  { acqStop();
    acqExit();

    check_hibernate(&parameters,1); 
  }
  
#if ON_TIME > 0
    #ifdef DO_LOGGER
      if(loopStatus==0) loopStatus=2;
    #endif
    
    if(millis()>(parameters.on_time*60+3)*1000)  //
    { doHibernate=1; 
      #ifdef DO_LOGGER
        loggerStop(1);
      #else
        loopStatus=1;
      #endif
    }
    
    #ifdef DO_LOGGER
      if(loopStatus==2){ if(!loggerLoop()) loopStatus=1; }
    #endif
      
#else
    int cmd=menu();
    switch(cmd)
    { case 1: if(loopStatus==0) acqStart(); loopStatus=1; break;
      #ifdef DO_LOGGER
        case 2: if(loopStatus==1) {loggerStart(); loopStatus=2;} break;
        case 3: if(loopStatus==2) {loggerStop(0);  loopStatus=2;} break;
      #endif
      case 4: if(loopStatus==1) {acqStop(); loopStatus=0;} break;
      case 5: if(loopStatus==2) 
              { doHibernate=1; 
                #ifdef DO_LOGGER
                  loggerStop(0);
                #endif
               }; break;
      case 9: while(1); break;
      default:
    	#ifdef DO_LOGGER
        if(loopStatus==2){ if(!loggerLoop()) loopStatus=1; }
    	#else
    		acqLoop();
    	#endif
    }
#endif
}

void printMenu(void)
{
  Serial.println();
  Serial.println("Menu");
  Serial.println("? Help");
  Serial.println("a Start Acquisition");
  Serial.println("b Start Logger");
  Serial.println("s Stop Logger");
  Serial.println("e End Aquisition");
  Serial.println("h End Aquisition and Hibernate");
  Serial.println("x Exit Program");
  Serial.println();
}

int menu(void)
{
  if(!Serial.available()) return 0; // no command
  char c=Serial.read();
  
  if (strchr("?xabseh", c))
  { switch (c)
    {
      case '?': printMenu(); return 0; // exit program
      case 'x': return 9; // exit program
      case 'a': return 1; // start acqisition
      case 'b': return 2; // start logger
      case 's': return 3; // stop logger (after finishing file)
      case 'e': return 4; // end aqusisition
      case 'h': return 5; // hibernate (after finishing file)
    }
  }
  return 0;
}

/**************** FOR Tim's Menu ***************************************/
#include <time.h>
static uint32_t getRTC(void) {return RTC_TSR;}
static void setRTC(uint32_t tt)
{
  RTC_SR = 0;
  RTC_TPR = 0;
  RTC_TSR = tt;
  RTC_SR = RTC_SR_TCE;
}

static char * getDate(char *text)
{
    uint32_t tt=getRTC();
    struct tm tx =seconds2tm(tt);
    sprintf(text,"%04d/%02d/%02d",tx.tm_year, tx.tm_mon, tx.tm_mday);
    return text;  
}

static char * getTime(char *text)
{
    uint32_t tt=getRTC();
    struct tm tx =seconds2tm(tt);
    sprintf(text,"%02d:%02d:%02d",tx.tm_hour, tx.tm_min, tx.tm_sec);
    return text;
}

static void setDate(uint16_t year, uint16_t month, uint16_t day)
{
    uint32_t tt=getRTC();
    struct tm tx=seconds2tm(tt);
    tx.tm_year=year;
    tx.tm_mon=month;
    tx.tm_mday=day;
    tt=tm2seconds(&tx);
    setRTC(tt);
}

static void setTime(uint16_t hour, uint16_t minutes, uint16_t seconds)
{
    uint32_t tt=getRTC();
    struct tm tx=seconds2tm(tt);
    tx.tm_hour=hour;
    tx.tm_min=minutes;
    tx.tm_sec=seconds;
    tt=tm2seconds(&tx);
    setRTC(tt);
}

/*   
  uint16_t on_time;
  uint16_t off_time;
  uint16_t first_hour;
  uint16_t second_hour;
  uint16_t third_hour;
  uint16_t last_hour;

 */
inline uint16_t check_time(uint16_t to, uint16_t t1, uint16_t t2)
{
  return (t1<t2)? (to>=t1)&&(to<t2) : (to>=t1)||(to<t2);
}

void check_hibernate( parameters_s *par, int flag)
{
  uint32_t tt=getRTC();
  struct tm tx=seconds2tm(tt);
  uint16_t to = tx.tm_hour;
  
  // check if we should sleep longer
  uint16_t doRecording=1;
  doRecording = check_time(to,par->first_hour, par->second_hour) ||
                check_time(to,par->third_hour, par->last_hour);
  
  int32_t dto=0, dt=0, t1;
  if(doRecording)
  { if(flag)
    {
      dto = (par->on_time + par->off_time)*60;
      dt = dto - (tt % dto);
      go_hibernate(dt);
  }
    else
      dto=0;
  }
  else
  { 
#define SLEEP_MODE 1  // 0: wakeup every recording period; 1: wakeup only at end of sleep time 
#if SLEEP_MODE==0
    dto = (par->on_time+par->off_time)*60;
    dt = dto - (tt % dto);
    go_hibernate(dt);
#else
    if(check_time(to,par->last_hour,par->first_hour))
    {
      dto=(par->first_hour - par->last_hour);
      t1=par->last_hour;
    }
    else // is in middle sleep period
    {
      dto=(par->third_hour - par->second_hour);
      t1=par->second_hour;
    }
    if(dto<0) dto += 24;

    dto *= 3600;
    t1 *= 3600;
    int32_t tx =tt % (24*3600); // seconds of day
    
    dt = dto - ((tx-t1 +24*3600) % (24*3600));
//    while(!Serial);  Serial.printf("%d %d %d %d\n\r",dto,t1,tx,dt); Serial.flush();  delay(10000);
    
    go_hibernate(dt);
#endif
  }
}

/*
? g\n:  ESM_Logger reports "on_time" value
? p\n:  ESM_Logger reports "off_time" value
? i\n:  ESM_Logger reports "first_hour" value
? u\n:  ESM_Logger reports "second_hour" value
? v\n:  ESM_Logger reports "third_hour" value
? f\n:  ESM_Logger reports "last_hour" value
? n\n:  ESM_Logger reports "name" value
? d\n:  ESM_Logger reports date
? t\n:  ESM_Logger reports time
? l\n:  ESM_Logger reports lux
? m\n:  ESM_Logger reports mac address (name?)
*/
char * encode_mac(char * text)  ;
int getLightSensor();

char text[32];

static void printAll(void)
{
  SERIALX.printf("%c %2d on_time\n\r",     'g',parameters.on_time);
  SERIALX.printf("%c %2d off_time\n\r",    'p',parameters.off_time);
  SERIALX.printf("%c %2d first_hour\n\r",  'i',parameters.first_hour);
  SERIALX.printf("%c %2d second_hour\n\r", 'u',parameters.second_hour);
  SERIALX.printf("%c %2d third_hour\n\r",  'v',parameters.third_hour);
  SERIALX.printf("%c %2d last_hour\n\r",   'f',parameters.last_hour);
  SERIALX.printf("%c %s name\n\r",         'n',parameters.name);
  SERIALX.printf("%c %s date\n\r",         'd',getDate(text));
  SERIALX.printf("%c %s time\n\r",         't',getTime(text));
  SERIALX.printf("%c %s mac address\n\r",  'm',encode_mac(text));
  SERIALX.println();
  SERIALX.println("exter 'a' to print this");
  SERIALX.println("exter '?c' to read value c=(g,p,i,u,v,f,n,d,t,m)");
  SERIALX.println("  e.g.: ?i will print first hour");
  SERIALX.println("exter '!cval' to read value c=(g,p,i,u,v,f,n,d,t) and val is new value");
  SERIALX.println("  e.g.: !i10 will set first hour to 10");
  SERIALX.println("exter 'xval' to exit menu (x is delay in minutes, -1 means immediate)");
  SERIALX.println("  e.g.: x10 will exit and hibernate for 10 minutes");
  SERIALX.println("        x-1 with exit and start immediately");
  SERIALX.println();
}

static void doMenu1(void)
{ // for enquiries
    while(!SERIALX.available());
    char c=SERIALX.read();
    
    if (strchr("gpiuvfndtlm", c))
    { switch (c)
      {
        case 'g': SERIALX.printf("%02d\r\n",parameters.on_time); break;
        case 'p': SERIALX.printf("%02d\r\n",parameters.off_time); break;
        case 'i': SERIALX.printf("%02d\r\n",parameters.first_hour);break;
        case 'u': SERIALX.printf("%02d\r\n",parameters.second_hour);break;
        case 'v': SERIALX.printf("%02d\r\n",parameters.third_hour);break;
        case 'f': SERIALX.printf("%02d\r\n",parameters.last_hour);break;
        case 'n': SERIALX.printf("%s\r\n",parameters.name);break;  // could be (unique) mac address
        case 'd': SERIALX.printf("%s\r\n",getDate(text));break;
        case 't': SERIALX.printf("%s\r\n",getTime(text));break;
        case 'l': SERIALX.printf("%04d\r\n",getLightSensor());break;
        case 'm': SERIALX.printf("%s\r\n",encode_mac(text)); break;
      }
    }
}

/*
! g val\n:       ESM_Logger sets "on_time" value 
! p val\n:       ESM_Logger sets "off_time" value
! i val\n:       ESM_Logger sets "first_hour" value
! u val\n:       ESM_Logger sets "second_hour" value
! v val\n:       ESM_Logger sets "third_hour" value
! f val\n:       ESM_Logger sets "last_hour" value
! n val\n:       ESM_Logger sets "name" value 
! d datestring\n ESM_Logger sets date
! t timestring\n ESM_Logger sets time
! x delay\n      ESM_Logger exits menu and hibernates for the amount given in delay
*/
static void doMenu2(void)
{ // for settings
    while(!SERIALX.available());
    char c=SERIALX.read();
    uint16_t year,month,day,hour,minutes,seconds;
    
    if (strchr("gpiuvfndt", c))
    { switch (c)
      {
        case 'g': parameters.on_time     =SERIALX.parseInt(); break;
        case 'p': parameters.off_time    =SERIALX.parseInt(); break;
        case 'i': parameters.first_hour  =SERIALX.parseInt();break;
        case 'u': parameters.second_hour =SERIALX.parseInt();break;
        case 'v': parameters.third_hour  =SERIALX.parseInt();break;
        case 'f': parameters.last_hour   =SERIALX.parseInt();break;
        case 'n': for(int ii=0; ii<4;ii++) parameters.name[ii] = SERIALX.read();
                  parameters.name[4]=0; break;
        case 'd':     
                  year= SERIALX.parseInt();
                  month= SERIALX.parseInt();
                  day= SERIALX.parseInt();
                  setDate(year,month,day);
                  break;
        case 't': 
                  hour= SERIALX.parseInt();
                  minutes= SERIALX.parseInt();
                  seconds= SERIALX.parseInt();
                  setTime(hour,minutes,seconds);
                  break;
      }
      parMods=1;
    }  
}

int16_t doMenu(void)
{
  int16_t ret=0;
  do
  {
    while(!SERIALX.available());
    char c=SERIALX.read();
    
    if (strchr("?!xa", c))
    { switch (c)
      {
        case '?': doMenu1(); break;
        case '!': doMenu2(); break;
        case 'x': ret = SERIALX.parseInt(); break;
        case 'a': printAll(); break;
      }
    }
  } while(ret==0);
  return ret;
}

/*********************** For reading Mac address ***********************/
uint8_t mac[6];

// http://forum.pjrc.com/threads/91-teensy-3-MAC-address
void doRead_mac(uint8_t word, uint8_t *mac, uint8_t offset) 
{
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF));

  *(mac+offset) =   FTFL_FCCOB5;       // collect only the top three bytes,
  *(mac+offset+1) = FTFL_FCCOB6;       // in the right orientation (big endian).
  *(mac+offset+2) = FTFL_FCCOB7;       // Skip FTFL_FCCOB4 as it's always 0.
}

void read_mac(void) 
{
  doRead_mac(0xe,mac,0);
  doRead_mac(0xf,mac,3);
}

char * encode_mac(char * text)  
{
  uint8_t ii,jj;
  uint8_t digit;
  read_mac();
  for(ii = 0,jj=0; ii < 6; ii++) 
  {
    if (ii!=0) text[jj++]='_';
    digit=(mac[ii] & 0xF0) >> 4;
    text[jj++]=((digit < 10) ? '0' + digit : 'A' + digit - 10);
    digit=(mac[ii] & 0x0F);
    text[jj++]=((digit < 10) ? '0' + digit : 'A' + digit - 10);
  }
  text[jj]=0;

  return text;
}

