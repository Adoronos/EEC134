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
#include <bcm2835.h>

 
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
 

#define NUMREADS 100

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


#define OLED_COMMANDLOCK    0xFD
#define OLED_DISPLAYOFF     0xAE
#define OLED_CLOCKDIV       0xB3
#define OLED_MUXRATIO       0xCA
#define OLED_SETREMAP       0xA0
#define OLED_SETCOLUMN      0x15
#define OLED_SETROW         0x75
#define OLED_STARTLINE      0xA1
#define OLED_DISPLAYOFFSET  0xA2
#define OLED_SETGPIO        0xB5
#define OLED_FUNCTIONSELECT 0xAB
#define OLED_PRECHARGE      0xB1
#define OLED_VCOMH          0xBE
#define OLED_NORMALDISPLAY  0xA6
#define OLED_CONTRASTABC    0xC1
#define OLED_CONTRASTMASTER 0xC7
#define OLED_SETVSL         0xB4
#define OLED_PRECHARGE2     0xB6
#define OLED_DISPLAYON      0xA5
#define OLED_WRITERAM       0x5C



void setup_io();

void writeData(uint8_t c);
void writeCommand(uint8_t d);


void goToPixel(int x, int y);



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
  int pins[4] = {11, 13, 16, 20, 21};
  unsigned int GPIODat = 0;
  int rst = 11;
  int dc = 13;

  bool pinDat;
  int ADCRead = 0;
  int data[NUMREADS];
  struct timespec tsucs, tsucs2;
  tsucs.tv_sec = 0;
  tsucs.tv_nsec = 90;




    // If you call this, it will not actually access the GPIO
// Use for testing
//        bcm2835_set_debug(1);
    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failedg. Are you running as root??\n");
      return 1;
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default


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
  for (g=0; g<5; g++)
  {
    INP_GPIO(pins[g]); // must use INP_GPIO before we can use OUT_GPIO
  }
 OUT_GPIO(clk);
 OUT_GPIO(cs);
 OUT_GPIO(rst);
 OUT_GPIO(dc);

    GPIO_SET = 1 << rst;
    sleep(1);
    GPIO_CLR = 1 << rst;
    sleep(1);
    GPIO_SET = 1 << rst;
    sleep(1);

    // Send a byte to the slave and simultaneously read a byte back from the slave
    // If you tie MISO to MOSI, you should read back what was sent
    writeCommand(OLED_COMMANDLOCK);
    writeData(0x12);
    writeCommand(OLED_COMMANDLOCK);
    writeData(0xB1);
    writeCommand(OLED_DISPLAYOFF);
    writeCommand(OLED_CLOCKDIV);
    writeData(0xF1);
    writeCommand(OLED_MUXRATIO);
    writeData(127);
    writeCommand(OLED_SETREMAP);
    writeData(0x74);
    writeCommand(OLED_SETCOLUMN);
    writeData(0x00);
    writeData(0x7F);
    writeCommand(OLED_SETROW);
    writeData(0x00);
    writeData(0x7F);
    writeCommand(OLED_STARTLINE);
    writeData(0);
    writeCommand(OLED_DISPLAYOFFSET);
    writeData(0x00);
    writeCommand(OLED_SETGPIO);
    writeData(0x00);
    writeCommand(OLED_FUNCTIONSELECT);
    writeData(0x01);
    writeCommand(OLED_PRECHARGE);
    writeData(0x32);
    writeCommand(OLED_VCOMH);
    writeData(0x05);
    writeCommand(OLED_NORMALDISPLAY);
    writeCommand(OLED_CONTRASTABC);
    writeData(0xC8);
    writeData(0x80);
    writeData(0xC8);
    writeCommand(OLED_CONTRASTMASTER);
    writeData(0x0F);
    writeCommand(OLED_SETVSL);
    writeData(0xA0);
    writeData(0xB5);
    writeData(0x55);
    writeCommand(OLED_PRECHARGE2);
    writeData(0x01);
    writeCommand(OLED_DISPLAYON);

    goToPixel(30, 30);

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


void goToPixel(int x, int y)
{
	writeCommand(OLED_SETCOLUMN);
	writeData(x);
	writeCommand(OLED_SETROW);
	writeData(y);

	writeCommand(OLED_WRITERAM);
}


void writeCommand(uint8_t c)
{
	GPIO_SET = 1 << dc;
	bcm2835_spi_transfer(c);
}

void writeData(uint8_t d)
{
	GPIO_CLR = 1 << dc;
	bcm2835_spi_transfer(d);
}
