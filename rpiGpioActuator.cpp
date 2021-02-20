
#include "httplib.h"

#define BCM2708_PERI_BASE       0x3F000000

#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/i2c-dev.h>	
#include <sys/ioctl.h>	
#include <linux/i2c.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

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

#define SENSOR 0x6d

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
extern "C" {
   #include <i2c/smbus.h>
}

#define I2C_ADAPTER "/dev/i2c-1"
#define I2C_DEVICE  0x6d

#include "linux/i2c-dev.h"



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


int delay(unsigned long mikros)
  {
  struct timespec ts;
  int err;

  ts.tv_sec = mikros / 1000000L;
  ts.tv_nsec = (mikros % 1000000L) * 1000L;
  err = nanosleep(&ts, (struct timespec *)NULL);
  return(err);
  }




int main(void)
{
  using namespace httplib;

  
  // Set up gpi pointer for direct register access
  setup_io();



  OUT_GPIO(7);
  OUT_GPIO(1);
  OUT_GPIO(12);
  OUT_GPIO(16);
  OUT_GPIO(20);

     GPIO_CLR = 1 << 3;
  int gpio_map[]={0, 7,1,12,16,20};



  Server svr;

  svr.Get("/hi", [](const Request& req, Response& res) {
    res.set_content("Hello World!", "text/plain");
  });
  svr.Get("/msr", [](const Request& req, Response& res) {


	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return;
	}
	
	int addr = 0x6d;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return;
	}
	//----- READ BYTES -----

int fd;
fd = open("/dev/i2c-1", O_RDWR);
char val[3];
ioctl(fd, I2C_SLAVE_FORCE, 0x6d);
    i2c_smbus_write_byte_data(fd, 0x30, 0x0b); // enable periodic meausure
  for(int i=0;i<3;i++)
{
   const __u8 reg = 0x6+i;
    // Using SMBus commands
    const __s32 result = i2c_smbus_read_byte_data(fd, reg);
    if (result < 0) {
         // ERROR HANDLING: i2c transaction failed
         printf("Oh dear, something went wrong with i2c_smbus_read_byte_data()>i2c_smbus_access()>ioctl()! %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    } else {
        // res contains the read word
    }
    val[i]=result;
}
close(fd);

   int value=val[0]<<16 |
	   val[1]<<8 |
 	   val[2] ;
   printf("Val %d \n", value);

    res.set_content("333Hello World!", "text/plain");
  });

  svr.Get("/abba", [](const Request& req, Response& res) {
    printf("Measure\n");
    for(int a=0 ; a<5;a++)
    {
     printf("%d\n", GET_GPIO(2)); 
    }
   int val=0;
    for(int i=0;i<24;i++)
    {
     GPIO_SET = 1 << 3;
    delay(1);
    if(GET_GPIO(2)==0) {
      val= val <<1;
    }else  {
      val= val <<1;
      val= val |  1;
     }
     GPIO_CLR = 1 << 3;
    delay(1);
    }
     printf("Val %d \t (0x%x)\n", val ,val);

     printf("\n");
    res.set_content("Hello World!", "text/plain");
  });
 svr.Get(R"(/on/(\d+))", [&](const Request& req, Response& res) {
    auto numbers = req.matches[1];
     int i = std::stoi (numbers);
     if(i>=1 && i<=6)
GPIO_SET = 1 << (gpio_map[i]);
    res.set_content(numbers, "text/plain");
  });
 svr.Get(R"(/off/(\d+))", [&](const Request& req, Response& res) {
    auto numbers = req.matches[1];
     int i = std::stoi (numbers);
     if(i>=1 && i<=6)
GPIO_CLR = 1 << gpio_map[i];
    res.set_content(numbers, "text/plain");
  });
   svr.listen("0.0.0.0", 8001);
}

