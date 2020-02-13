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

    int serial_port = open("/dev/ttyUSB1", O_RDWR);
    // Check id success
    if(serial_port < 0)
    {
        std::cout << "Error " << errno << " from open: " << strerror(errno) << std::endl;
    }

    return 0;
}
