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

#define ECHO_BUFFER_SIZE 100
#include <gpiod.h> // Used only to detect the API version.
#include <gpiod.hpp>
#include <exception>
#include <optional>
#define	CONSUMER "smartServo_driver"

// libgpiod has no compile-time version macro. This public bulk API macro
// exists in 1.x and was removed together with the old line API in 2.x.
#ifdef GPIOD_LINE_BULK_MAX_LINES
#define SMART_SERVO_GPIOD_V1 1
#else
#define SMART_SERVO_GPIOD_V1 0
#endif

namespace {
bool useRpi = true;
#if SMART_SERVO_GPIOD_V1
std::optional<gpiod::line> driverGPIO;
#else
std::optional<gpiod::line_request> driverGPIO;
unsigned int driverOffset = 0;
#endif
}



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


void initDriver(const char *chip_path, int gpio, bool rpi){
    driverGPIO.reset();
    useRpi = rpi;
    if (!useRpi) return;
    if (gpio < 0) {
        fprintf(stderr, "Invalid GPIO offset %d\n", gpio);
        return;
    }
    try {
        // The driver is active-low: start disabled (physical high).
        #if SMART_SERVO_GPIOD_V1
        auto line = gpiod::chip(chip_path, gpiod::chip::OPEN_BY_PATH).get_line(gpio);
        line.request({CONSUMER, gpiod::line_request::DIRECTION_OUTPUT, {}}, 1);
        driverGPIO = line;
        #else
        driverOffset = static_cast<unsigned int>(gpio);
        driverGPIO = gpiod::chip(chip_path)
            .prepare_request()
            .set_consumer(CONSUMER)
            .add_line_settings(driverOffset, gpiod::line_settings()
                .set_direction(gpiod::line::direction::OUTPUT)
                .set_output_value(gpiod::line::value::ACTIVE))
            .do_request();
        #endif
    } catch (const std::exception &error) {
        fprintf(stderr, "Error initializing driver GPIO %s line %d: %s\n", chip_path, gpio, error.what());
    }
}

void enableDriver(int fd, bool enable){
    if (!useRpi) {
        int rts = TIOCM_RTS;
        ioctl(fd, enable ? TIOCMBIS : TIOCMBIC, &rts);
        return;
    }
    if (!driverGPIO) return;
    try {
        #if SMART_SERVO_GPIOD_V1
        driverGPIO->set_value(!enable);
        #else
        driverGPIO->set_value(driverOffset,
            enable ? gpiod::line::value::INACTIVE : gpiod::line::value::ACTIVE);
        #endif
    } catch (const std::exception &error) {
        fprintf(stderr, "Error setting driver GPIO: %s\n", error.what());
    }

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
