// C libray Header
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux Header
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

char read_buf_g [512];
// write buffer

int serial_send(int port, std::string tx_data) {
    tx_data.append("\r\n");
    int tx_num = write(port, tx_data.c_str(), tx_data.size());

    return tx_num;
}

int serial_read(int port, std::string& rx_data) {
    rx_data.clear();
    int rx_num = read(port, &read_buf_g, sizeof(read_buf_g));
    rx_data = std::string(&read_buf_g[0], rx_num);

    // The last character of a package
    //std::size_t lf_pos = rx_data.find_first_of('\n');
    // Position before the last character of a package
    //std::size_t cr_pos = rx_data.find_first_of('\r', lf_pos);
    std::size_t curr_pos = 0;
    std::size_t lf_pos = rx_data.find_first_of('\n', curr_pos);
    rx_data = rx_data.substr(lf_pos);
    for(size_t pos_itr = 0; pos_itr < rx_data.size(); pos_itr++) {
        std::size_t lf_pos = rx_data.find_first_of('\n', curr_pos);
        std::size_t cr_pos = rx_data.find_first_of('\r', lf_pos+1);
        if( (lf_pos != std::string::npos) && (cr_pos != std::string::npos) && (cr_pos - lf_pos == 86)) {
            curr_pos = cr_pos;
            std::string disp_data = rx_data.substr(lf_pos+2, 84);
            std::size_t gyro_x_pos = disp_data.find_first_of('\t');
            std::size_t gyro_y_pos = disp_data.find_first_of('\t', gyro_x_pos+1);
            std::size_t gyro_z_pos = disp_data.find_first_of('\t', gyro_y_pos+1);
            std::size_t acc_x_pos = disp_data.find_first_of('\t', gyro_z_pos+1);
            std::size_t acc_y_pos = disp_data.find_first_of('\t', acc_x_pos+1);
            std::size_t acc_z_pos = disp_data.find_first_of('\t', acc_y_pos+1);
            std::size_t status_pos = disp_data.find_first_of('\t', acc_z_pos+1);
            //std::cout   << rx_data.substr(lf_pos+2, 84)

            std::cout   << disp_data.substr(0,gyro_x_pos) << "|"
                        << disp_data.substr(gyro_x_pos+1,10) << "|"
                        << disp_data.substr(gyro_y_pos+1,10) << "|"
                        << disp_data.substr(gyro_z_pos+1,10) << "|"
                        << disp_data.substr(acc_x_pos+1,10) << "|"
                        << disp_data.substr(acc_y_pos+1,10) << "|"
                        << disp_data.substr(acc_z_pos+1,10) << "|"
                        << disp_data.substr(status_pos) << "|"

                        << ": " << lf_pos
                        << ": " << cr_pos
                        << ": " << cr_pos - lf_pos
                        << std::endl;
        }else {
            break;
        }


    }
    //std::cout << "##########################################################" << std::endl;
/*
    std::cout   << rx_data.substr(lf_pos+2, 84)
                << ": " << lf_pos
                << ": " << cr_pos
                << ": " << cr_pos - lf_pos << std::endl;
*/
    //std::cout << "##########################################################" << std::endl;

    return rx_num;
}

int main(int argc, char **argv) {
    std::cout << "### IMU Parser ###" << std::endl;

    const char device_name[] = "/dev/ttyUSB0";
    // Allocate memory for read buffer, set size according to your needs
    memset(&read_buf_g, '\0', sizeof(read_buf_g));

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

    /*### Start Read / Write operation with IMU ###*/
    serial_send(serial_port, "stop") ;
    serial_send(serial_port, "meas") ;

    std::string data;
    while(1) {
        serial_read(serial_port, data);
    }
    //std::cout << data << std::endl;
/*
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
    std::string data;
    data = std::string(&read_buf[0], num_bytes);
    std::cout << data << std::endl;
    data.clear();
*/
    serial_send(serial_port, "stop") ;






    /*### End Read / Write operation with IMU ###*/
    // Close port after use
    std::cout << "Close serial port: " << device_name << std::endl;
    close(serial_port);
    return 0;
}
