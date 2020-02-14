// C libray Header
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux Header
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <signal.h> // For keyboard interrupt

volatile sig_atomic_t gSignalStatus;
char read_buf_g [512];

void inthand(int signum) {
    gSignalStatus = 1;
}

int serial_send(int port, std::string tx_data) {
    tx_data.append("\r\n");
    //std::cout << "serial_send:" << tx_data << "--" << tx_data.size() << std::endl;
    int tx_num = write(port, tx_data.c_str(), tx_data.size());

    return tx_num;
}

int serial_read(int port, std::string& rx_data) {
    // Block read
    rx_data.clear();
    int rx_num = read(port, &read_buf_g, sizeof(read_buf_g));
    rx_data = std::string(&read_buf_g[0], rx_num);

    return rx_num;
}

void process_Data(std::string& rx_data) {
    // Discard every string before the first ''\n'
    std::size_t curr_pos = 0;
    std::size_t lf_pos = rx_data.find_first_of('\n', curr_pos);
    rx_data = rx_data.substr(lf_pos);

    std::string::size_type sz;     // alias of size_t
    for(size_t pos_itr = 0; pos_itr < rx_data.size(); pos_itr++)
    {
        std::size_t lf_pos = rx_data.find_first_of('\n', curr_pos);
        std::size_t cr_pos = rx_data.find_first_of('\r', lf_pos+1);
        if( (lf_pos != std::string::npos) && (cr_pos != std::string::npos) && (cr_pos - lf_pos == 86)) {
            curr_pos = cr_pos;
            std::string disp_data = rx_data.substr(lf_pos+2, 84);

            double time_stamp = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double gyro_x = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double gyro_y = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double gyro_z = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double acc_x = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double acc_y = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double acc_z = std::stod(disp_data, &sz);
            disp_data = disp_data.substr(sz);
            double status = std::stoi(disp_data, &sz);
            disp_data = disp_data.substr(sz);

            std::cout << time_stamp << "|" << gyro_x << "|" << gyro_y
            << "|" << gyro_z << "|" << acc_x << "|" << acc_y
            << "|" << acc_z << "|" << status << std::endl;

        } else {
            break;
        }
    }
}

int main(int argc, char **argv) {
    std::cout << "### IMU Parser ###" << std::endl;
    signal(SIGINT, inthand);

    const char device_name[] = "/dev/ttyUSB0";
    // Allocate memory for read buffer, set size according to your needs
    memset(&read_buf_g, '\0', sizeof(read_buf_g));
    std::string data;

    int serial_port = open(device_name, O_RDWR);
    // Check id success
    if(serial_port < 0)
    {
        std::cout << "Error " << errno << " from open serial port: " << strerror(errno) << std::endl;
    }else{
        std::cout << "Open device: " << device_name << " success!" << std::endl;
        tcflush(serial_port,TCIOFLUSH);
    }

    // termios struct name config
    struct termios config;

    // Read in existing setting
    if(tcgetattr(serial_port, &config) != 0)
    {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    //
    // Input flags
    config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP|
                        INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

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
    config.c_oflag &= ~(OPOST | ONLCR);

    //
    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    //
    // Turn off character processing
    //
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    config.c_cflag |= CS8 | CREAD | CLOCAL;

    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN]  = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    config.c_cc[VTIME] = 1;

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
    serial_send(serial_port, "meas");
    while(!gSignalStatus) {
        serial_read(serial_port, data);
        process_Data(data);
    }

    /*### End Read / Write operation with IMU ###*/
    // Close port after use
    serial_send(serial_port, "stop");
    sleep(2);
    tcflush(serial_port,TCIOFLUSH);
    std::cout << "Close serial port: " << device_name << std::endl;
    close(serial_port);
    return 0;
}
