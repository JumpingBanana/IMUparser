// C libray Header
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux Header
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int main(int argc, char **argv)
{
    std::cout << "### IMU Parser ###" << std::endl;

    const char device_name[] = "/dev/ttyUSB1";
    // Allocate memory for read buffer, set size according to your needs
    char read_buf [256];
    memset(&read_buf, '\0', sizeof(read_buf));
    // write buffer
    unsigned char msg[] = { 'm', 'e', 'a', 's', '\r' };

    int serial_port = open(device_name, O_RDWR);
    // Check id success
    if(serial_port < 0)
    {
        std::cout << "Error " << errno << " from open serial port: " << strerror(errno) << std::endl;
    }else{
        std::cout << "Open device: " << device_name << " success!" << std::endl;
    }

    // termios struct name config
    struct termios config;

    // Read in existing setting
    if(tcgetattr(serial_port, &config) != 0)
    {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    //
    // Input flags - Turn off input processing
    //
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);

    //
    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config.c_oflag = 0;

    //
    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    //
    // Turn off character processing
    //
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN]  = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    config.c_cc[VTIME] = 0;

    //
    // Communication speed (simple version, using the predefined
    // constants)
    //
    if(cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0) {
        std::cout << "Error " << errno << " from cfsetispeed: " << strerror(errno) << std::endl;
    }

    //
    // Finally, apply the configuration
    //
    if(tcsetattr(serial_port, TCSAFLUSH, &config) < 0) {
        std::cout << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
    }


    write(serial_port, msg, sizeof(msg));

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
    printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

    // Close port after use
    std::cout << "Close serial port: " << device_name << std::endl;
    close(serial_port);

    return 0;
}
