#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()
#include <linux/serial.h>
#include <asm/termbits.h>

// #define RPI
#define ECHO_BUFFER_SIZE 100
#if RPI
#include <gpiod.h>
#define GPIO_CHIP "gpiochip4"
#define	CONSUMER "smartServo_driver"
struct gpiod_line* line;
#endif



//int ioctl(int fd, unsigned long op, ...);

int init_serial(int fd, speed_t speed) {
    // Create new termios struct, we call it 'tty' for convention
  struct termios2 tty;

  // Read in existing settings, and handle any error
  //if(tcgetattr(fd, &tty) != 0) {
  if(ioctl(fd, TCGETS2, &tty) != 0) {
      printf("Error %i from ioctl TCGETS2: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Set non standard baudrate
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = speed;
  tty.c_ospeed = speed;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 1;    // Wait for up to 0.1s (1 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;


  // Save tty settings, also checking for error
  if(ioctl(fd, TCSETS2, &tty) != 0) {
      printf("Error %i from ioctl TCSETS2: %s\n", errno, strerror(errno));
      return 1;
  }
  return 0;
}


void initDriver(int gpio){
    #if RPI
    struct gpiod_chip *chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        std::cout << "Open chip failed\n";
        return -1;
    }

    line = gpiod_chip_get_line(chip, gpio);

    if(gpiod_line_request_output(line, CONSUMER, 0)) {
       std::cout << "Error requesting line " << gpio;
       return;
    }
    gpiod_line_set_value(line, 1);
    #endif
}

void enableDriver(int fd, bool enable){
    #if RPI
    gpiod_line_set_value(line, enable);
    #else
    int rts = TIOCM_RTS;
    if (enable){
        ioctl(fd,TIOCMBIS, &rts);
    }
    else{
        ioctl(fd,TIOCMBIC, &rts);
    }
    #endif
}

int writeData(int fd, uint8_t* data, size_t len, bool echo) {
    static uint8_t buffer[ECHO_BUFFER_SIZE];
    if(echo &&len > ECHO_BUFFER_SIZE )  {
        printf("Error: Cannot read echo: len > ECHO_BUFFER_SIZE\n");
        return -1;
    }

    enableDriver(fd, true);
    ssize_t len_write = write(fd, data, len);
    if(len_write != len) {
	printf("Write Error: %lu/%lu bytes written.\n", len_write, len);
        return -1;
    }
    


    // wait for the transmit buffer to be empty
    // seems to be working on Rpi with hardware uart, but not with an USB<->UART dongle
    uint8_t lsr;
    do {
      int r = ioctl(fd, TIOCSERGETLSR, &lsr);
    } while (!(lsr & TIOCSER_TEMT));

    enableDriver(fd, false);

    if (echo)
    {
        ssize_t echo_len = read(fd, buffer, len);
        if (len == echo_len && !memcmp(buffer, data, len)) {
	    //printf("echo ok!\n");
            //ok
        } else {
            printf("Error: Echo does not match\n");
//            tcflush(fd, TCIOFLUSH); // empty buffer
            usleep(100000); // wait 100ms
            return -1;
        }
    }

    return 0;
}

