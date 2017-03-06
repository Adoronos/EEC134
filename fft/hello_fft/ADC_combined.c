//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013
 
 
// Access from ARM Running Linux
 
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
 
 
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

#include "mailbox.h"
#include "gpu_fft.h"

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
 

#define NUMREADS 4096



char Usage[] =
    "Usage: hello_fft.bin log2_N [jobs [loops]]\n"
    "log2_N = log2(FFT_length),       log2_N = 8...22\n"
    "jobs   = transforms per batch,   jobs>0,        default 1\n"
    "loops  = number of test repeats, loops>0,       default 1\n";

unsigned Microseconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec*1000000 + ts.tv_nsec/1000;
}


int  mem_fd;
void *gpio_map;
 
// I/O access
volatile unsigned *gpio;
 
 
// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
 
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
 
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock
#define GPIO_LEV *(gpio+13) 

void setup_io();
void DelayMicrosecondsNoSleep(int delay_us);

 
void printButton(int g)
{
  if (GET_GPIO(g)) // !=0 <-> bit is 1 <- port is HIGH=3.3V
    printf("Button pressed!\n");
  else // port is LOW=0V
    printf("Button released!\n");
}
 
int main(int argc, char **argv)
{
  int g,rep;
  int clk  = 16;
  int dout = 20;
  int cs   = 21;
  int pins[3] = {16, 20, 21};
  unsigned int GPIODat = 0;
  bool pinDat;
  int ADCRead = 0;
  volatile int data[NUMREADS + 1];
  struct timespec tsucs, tsucs2;
  tsucs.tv_sec = 0;
  tsucs.tv_nsec = 90;
  struct timespec fftWait, fftWait2;
  fftWait.tv_sec = 0;
  fftWait.tv_nsec = 1000;
  volatile double output[NUMREADS + 1];

  int i, j, k, ret, loops, freq, log2_N, jobs, N, mb = mbox_open();
  unsigned t[2];
  double tsq[2];

  struct GPU_FFT_COMPLEX *base;
  struct GPU_FFT *fft;
 
  int start;
  int end;
  int timeLength;
  jobs = 1;


    N = 1<<12; // FFT length
    ret = gpu_fft_prepare(mb, 12, GPU_FFT_FWD, 1, &fft); // call once

    switch(ret) {
        case -1: printf("Unable to enable V3D. Please check your firmware is up to date.\n"); return -1;
        case -2: printf("log2_N=%d not supported.  Try between 8 and 22.\n", log2_N);         return -1;
        case -3: printf("Out of memory.  Try a smaller batch or increase GPU memory.\n");     return -1;
        case -4: printf("Unable to map Videocore peripherals into ARM memory space.\n");      return -1;
        case -5: printf("Can't open libbcm_host.\n");                                         return -1;
   }



  // Set up gpi pointer for direct register access
  setup_io();
 
  // Switch GPIO 7..11 to output mode
 
 /************************************************************************\
  * You are about to change the GPIO settings of your computer.          *
  * Mess this up and it will stop working!                               *
  * It might be a good idea to 'sync' before running this program        *
  * so at least you still have your code changes written to the SD-card! *
 \************************************************************************/
 
  // Set GPIO pins 7-11 to output
  for (g=0; g<3; g++)
  {
    INP_GPIO(pins[g]); // must use INP_GPIO before we can use OUT_GPIO
  }


printf("Start\n");
 OUT_GPIO(clk);
 OUT_GPIO(cs);

start = Microseconds();
  for (rep=0; rep<NUMREADS; rep++)
  {
     GPIO_SET = 1 << clk;
     GPIO_CLR = 1 << cs;
     nanosleep(&tsucs, &tsucs2);
     GPIO_CLR = 1 << clk;

     for (g=0; g<6; g++)
     {
       GPIO_SET = 1 << clk;
       GPIO_CLR = 1 << clk;
     }

     for (g=0; g<16; g++)
     {
       GPIO_SET = 1 << clk;
       GPIODat = GPIO_LEV;
       pinDat  = ((GPIODat & (1 << dout)) != 0);
       ADCRead = (ADCRead | (pinDat << (15-g)));
       GPIO_CLR = 1 << clk;
     }

     data[rep] = ADCRead;
     ADCRead = 0;
     GPIO_SET = 1 << cs;

     for (g=0; g<3; g++)
     {
      GPIO_SET = 1 << clk;
      GPIO_CLR = 1 << clk;
     }


    // printf("Data: %d %X\n", rep, data[rep]);
}


end = Microseconds();
timeLength = end-start;
printf("%d\n", timeLength);



   for (j=0; j<jobs; j++) {
        base = fft->in + j*fft->step; // input buffer
        for (i=0; i<N; i++)
	{
	    base[i].re = base[i].im = 0;
            base[i].re = data[i];
	   // printf("Val: %d %f %f\n", i, base[i].re, base[i].im);
	}
   }

    nanosleep(&fftWait, &fftWait2); // Yield to OS
    t[0] = Microseconds();
    gpu_fft_execute(fft); // call one or many times
    t[1] = Microseconds();



    tsq[0]=tsq[1]=0;
    for (j=0; j<jobs; j++) {
        base = fft->out + j*fft->step; // output buffer
        freq = j+1;
        for (i=0; i<N; i++) {
            output[i] = pow(base[i].re, 2) + pow(base[i].im, 2);
	   // printf("FFT: %f\n", output);
        }
    }



    printf("rel_rms_err = %0.2g, usecs = %d, k = %d\n",
            sqrt(tsq[1]/tsq[0]), (t[1]-t[0])/jobs, k);


    gpu_fft_release(fft); // Videocore memory lost if not freed !

 
  return 0;

} // main


//
// Set up a memory regions to access GPIO
//
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }
 
   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );
 
   close(mem_fd); //No need to keep mem_fd open after mmap
 
   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }
 
   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;
 
 
} // setup_io


/*
void DelayMicrosecondsNoSleep(int delay_us)
{
   long int start_time;
   long int time_difference;
   struct timespec gettime_now;

   clock_gettime(CLOCK_REALTIME, &gettime_now);
   start_time = gettime_now.tv_nsec;
   while (1)
   {
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	time_difference = gettime_now.tv_nsec - start_time;
	if (time_difference < 0)
		time_difference += 1000000000;
	if (time_difference > (delay_us * 1000))
		break;

   }


}
*/
