# IMUparser
Read IMU data via UART and parse data to get acceleration and angular rate.

## LXC setup
To add ttyUSB device to LXC containner, use the following command from host PC:

`lxc config device add [containner name] [device name] unix-char path=[/dev/ttyUSB1] mode=0666 gid=20`

for example:

`lxc config device add PythonAI ttyUSB1 unix-char path=/dev/ttyUSB1 mode=0666 gid=20`

The important past is the gid=20, which tell the containner that we would like to add
the device as 'dialout' group, and not 'root' group. If the /dev/ttyUSB1 was added
to 'root' group, we will need to use 'sudo' everytime we want to access the device.

At the end we should have this in the containner environment.
> $ls -la /dev/ttyUSB1

and result

> crw-rw-rw- 1 root dialout 188, 1 Feb 13 06:02 /dev/ttyUSB1

## Reference site
### CMake:
Prepare CMakeLists.txt
- https://cmake.org/cmake/help/latest/guide/tutorial/index.html

### Serial port:
- [linux serial ports using c++](https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/)
- Don't forget to add youself to dialout group: `sudo adduser $USER dialout`
- Using `termios` : https://en.wikibooks.org/wiki/Serial_Programming/termios
- C++ style of implementation: https://github.com/gbmhunter/CppLinuxSerial/blob/master/src/SerialPort.cpp
- GNU Setting Terminal modes : [GNU](http://www.gnu.org/software/libc/manual/html_node/Terminal-Modes.html)
