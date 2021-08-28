
// g++ -li2c rpiGpioActuator.cpp 
#include "httplib.h"

// Rpi 3 #define BCM2708_PERI_BASE       0x3F000000
// Rpi 4
 #define BCM2708_PERI_BASE 0xFE000000

#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <chrono>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

int mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or
// SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *(gpio + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define SET_GPIO_ALT(g, a)                                                     \
  *(gpio + (((g) / 10))) |=                                                    \
      (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define GPIO_SET *(gpio + 7) // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR                                                               \
  *(gpio + 10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio + 13) & (1 << g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio + 37)     // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio + 38) // Pull up/pull down clock

#define SENSOR 0x6d

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
extern "C" {
#include <i2c/smbus.h>
}

#define I2C_ADAPTER "/dev/i2c-1"
#define I2C_DEVICE 0x6d

#include "linux/i2c-dev.h"

//
// Set up a memory regions to access GPIO
//
void setup_io() {
  /* open /dev/mem */
  if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
    printf("can't open /dev/mem \n");
    exit(-1);
  }

  /* mmap GPIO */
  gpio_map =
      mmap(NULL,                   // Any adddress in our space will do
           BLOCK_SIZE,             // Map length
           PROT_READ | PROT_WRITE, // Enable reading & writting to mapped memory
           MAP_SHARED,             // Shared with other processes
           mem_fd,                 // File to map
           GPIO_BASE               // Offset to GPIO peripheral
      );

  close(mem_fd); // No need to keep mem_fd open after mmap

  if (gpio_map == MAP_FAILED) {
    printf("mmap error %d\n", (int)gpio_map); // errno also set!
    exit(-1);
  }

  // Always use volatile pointer!
  gpio = (volatile unsigned *)gpio_map;

} // setup_io

int delay(unsigned long mikros) {
  struct timespec ts;
  int err;

  ts.tv_sec = mikros / 1000000L;
  ts.tv_nsec = (mikros % 1000000L) * 1000L;
  err = nanosleep(&ts, (struct timespec *)NULL);
  return (err);
}

static volatile int value = 0;
  int fd;

int measure(int c) {
  int length;
  int result;
  unsigned char buffer[60] = {0};

  ioctl(fd, I2C_SLAVE_FORCE, 0x70);
  if(c==0)
    result=i2c_smbus_write_byte_data(fd, 0,4);
  else
    result= i2c_smbus_write_byte_data(fd, 0,5);

   if (result < 0) {
      printf("Error switching I2C channel\n");
      return 0;
   }

  int addr = 0x6d; //<<<<<The I2C address of the slave
  if (ioctl(fd, I2C_SLAVE, addr) < 0) {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    // ERROR HANDLING; you can check errno to see what went wrong
    return 0;
  }
  i2c_smbus_write_byte_data(fd, 0x30, 0x0b); // enable periodic meausure
  //----- READ BYTES -----
  char val[3];

  for (int i = 0; i < 3; i++) {
    const __u8 reg = 0x6 + i;
    // Using SMBus commands
    result = i2c_smbus_read_byte_data(fd, reg);
    if (result < 0) {
      // ERROR HANDLING: i2c transaction failed
      printf("Oh dear, something went wrong with "
             "i2c_smbus_read_byte_data()>i2c_smbus_access()>ioctl()! %s\n",
             strerror(errno));
      exit(EXIT_FAILURE);
    } else {
      // res contains the read word
    }
    val[i] = result;
    printf("res %d %x\n", i, result);
  }

  value = val[0] << 16 | val[1] << 8 | val[2];
    printf("res  %02x %02x %02x \n", val[0],val[1],val[2]);

  return value;
}

bool pump=false;
bool m0=false;
bool m1=false;

int main(void) {
  using namespace httplib;
  fd = open("/dev/i2c-1", O_RDWR);
  ioctl(fd, I2C_SLAVE_FORCE, 0x6d);
  i2c_smbus_write_byte_data(fd, 0x30, 0x0b); // enable periodic meausure

  // Set up gpi pointer for direct register access
  setup_io();


  OUT_GPIO(7);
  OUT_GPIO(1);
  OUT_GPIO(12);
  OUT_GPIO(16);
  OUT_GPIO(20);

  GPIO_CLR = 1 << 3;
  int gpio_map[] = {0, 7, 1, 12, 16, 20};

  int valve0=0;
  int valve1=0;
  int onTimeout=10;

  std::thread offTimeout([&]() {
    while (1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
       if(onTimeout<=0)
       {
          onTimeout=0;
          GPIO_CLR = 1 << (gpio_map[0]);
          GPIO_CLR = 1 << (gpio_map[1]);
          GPIO_CLR = 1 << (gpio_map[2]);
          GPIO_CLR = 1 << (gpio_map[3]);
          GPIO_CLR = 1 << (gpio_map[4]);
       }else {
	 onTimeout--;
       }
    }
  });
  std::thread t([&]() {
    while (1) {
      if(m0)  {
         valve0 = measure(0);
      }
      if(m1)  {
         valve1 = measure(1);
      }

      if(pump) 
      {
      int cur = measure(0);
      if(cur<10413)
        GPIO_SET = 1 << (gpio_map[4]);
      else if (cur > 16377216)
        GPIO_SET = 1 << (gpio_map[4]);
      else if (cur < 15067437)
        GPIO_CLR = 1 << (gpio_map[4]);
      }else {
        GPIO_CLR = 1 << (gpio_map[4]);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  });

  Server svr;

  svr.Get("/hi", [](const Request &req, Response &res) {
    res.set_content("Hello World!", "text/plain");
  });
  svr.Get("/v/0", [&](const Request &req, Response &res) {
    m0=true;
    res.set_content("pump:" + std::to_string(valve0), "text/plain");
  });
  svr.Get("/v/1", [&](const Request &req, Response &res) {
    m1=true;
    res.set_content("pump:" + std::to_string(valve1), "text/plain");
  });
  svr.Get("/msr", [](const Request &req, Response &res) {
    res.set_content("pump:" + std::to_string(value), "text/plain");
  });

  svr.Get("/pumpON", [&](const Request &req, Response &res) {
    pump=true;
    GPIO_SET = 1 << (gpio_map[4]);
    res.set_content("Pump ON", "text/plain");
  });
  svr.Get("/pumpOFF", [&](const Request &req, Response &res) {
    pump=false;
    GPIO_CLR = 1 << (gpio_map[4]);
    res.set_content("Pump OFF", "text/plain");
  });
  svr.Get(R"(/on/(\d+))", [&](const Request &req, Response &res) {
    auto numbers = req.matches[1];
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    onTimeout=10;
    int i = std::stoi(numbers);
    if (i >= 1 && i <= 6)
      GPIO_SET = 1 << (gpio_map[i]);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    res.set_content(numbers, "text/plain");
  });
  svr.Get(R"(/off/(\d+))", [&](const Request &req, Response &res) {
    auto numbers = req.matches[1];
    int i = std::stoi(numbers);
    if (i >= 1 && i <= 6)
      GPIO_CLR = 1 << gpio_map[i];
    res.set_content(numbers, "text/plain");
  });
  svr.listen("0.0.0.0", 8001);
}
