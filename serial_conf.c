#include <stdbool.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "serial_conf.h"

int setPortOptions(int fd, int baudrate, int databits, int parity, int stop,
        bool softwareHandshake, bool hardwareHandshake) {
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    if (tcgetattr(fd, &newtio) != 0) {
        return ERR_TC_GET_ATTR;
    }

    speed_t _baud= -1;
    switch (baudrate) {
#ifdef B0
        case       0: _baud= B0;       break;
#endif
#ifdef B50
        case      50: _baud= B50;      break;
#endif
#ifdef B75
        case      75: _baud= B75;      break;
#endif
#ifdef B110
        case     110: _baud= B110;     break;
#endif
#ifdef B134
        case     134: _baud= B134;     break;
#endif
#ifdef B150
        case     150: _baud= B150;     break;
#endif
#ifdef B200
        case     200: _baud= B200;     break;
#endif
#ifdef B300
        case     300: _baud= B300;     break;
#endif
#ifdef B600
        case     600: _baud= B600;     break;
#endif
#ifdef B1200
        case    1200: _baud= B1200;    break;
#endif
#ifdef B1800
        case    1800: _baud= B1800;    break;
#endif
#ifdef B2400
        case    2400: _baud= B2400;    break;
#endif
#ifdef B4800
        case    4800: _baud= B4800;    break;
#endif
#ifdef B7200
        case    7200: _baud= B7200;    break;
#endif
#ifdef B9600
        case    9600: _baud= B9600;    break;
#endif
#ifdef B14400
        case   14400: _baud= B14400;   break;
#endif
#ifdef B19200
        case   19200: _baud= B19200;   break;
#endif
#ifdef B28800
        case   28800: _baud= B28800;   break;
#endif
#ifdef B38400
        case   38400: _baud= B38400;   break;
#endif
#ifdef B57600
        case   57600: _baud= B57600;   break;
#endif
#ifdef B76800
        case   76800: _baud= B76800;   break;
#endif
#ifdef B115200
        case  115200: _baud= B115200;  break;
#endif
#ifdef B128000
        case  128000: _baud= B128000;  break;
#endif
#ifdef B230400
        case  230400: _baud= B230400;  break;
#endif
#ifdef B460800
        case  460800: _baud= B460800;  break;
#endif
#ifdef B576000
        case  576000: _baud= B576000;  break;
#endif
#ifdef B921600
        case  921600: _baud= B921600;  break;
#endif
#ifdef B1000000
        case 1000000: _baud= B1000000; break;
#endif
#ifdef B1152000
        case 1152000: _baud= B1152000; break;
#endif
#ifdef B1500000
        case 1500000: _baud= B1500000; break;
#endif
#ifdef B2000000
        case 2000000: _baud= B2000000; break;
#endif
#ifdef B2500000
        case 2500000: _baud= B2500000; break;
#endif
#ifdef B3000000
        case 3000000: _baud= B3000000; break;
#endif
#ifdef B3500000
        case 3500000: _baud= B3500000; break;
#endif
#ifdef B4000000
        case 4000000: _baud= B4000000; break;
#endif
        default:
            return ERR_BAUD;
    }

    if (_baud != -1) {
        cfsetospeed(&newtio, _baud);
        cfsetispeed(&newtio, _baud);
    }

    switch (databits) {
        case 5:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
            break;
        case 6:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
            break;
        case 7:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
            break;
        case 8:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
            break;
        default:
            return ERR_BITS;
    }

    // ignore modem control lines and enable the receiver
    newtio.c_cflag |=  CLOCAL | CREAD;

    // parity (0 == no parity, 1 == odd, 2 == even)
    newtio.c_cflag &= ~(PARENB | PARODD);
    if (parity == 2) {
        newtio.c_cflag |= PARENB;
    } else if (parity == 1) {
        newtio.c_cflag |= (PARENB | PARODD);
    } else if (parity != 0) {
        return ERR_PARITY;
    }

    // Set 1 stopbit
    newtio.c_cflag &= ~CSTOPB;

    if (stop == 2) {
        newtio.c_cflag |= CSTOPB;
    } else if(stop != 1) {
        return ERR_STOP_BITS;
    }

    // Ignore "BREAK" condition on input
    newtio.c_iflag= IGNBRK;

    // Enable or disabled the software handshake
    if (softwareHandshake) {
        newtio.c_iflag |= IXON | IXOFF;
    } else {
        newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    }

    // clear out the local and output modes
    newtio.c_lflag= 0;
    newtio.c_oflag= 0;

    // Set timeout to 0
    newtio.c_cc[VTIME]= 0;

    // 1 char minimum for a non-canonical read
    newtio.c_cc[VMIN]= 1;

    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
        return ERR_TC_SET_ATTR;
    }

    int mcs= 0;
    ioctl(fd, TIOCMGET, &mcs);
    mcs |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &mcs);

    if (tcgetattr(fd, &newtio) != 0) {
        return ERR_TC_GET_ATTR;
    }

#ifdef CRTSCTS
    // hardware handshake
    if (hardwareHandshake) {
        newtio.c_cflag |= CRTSCTS;
    } else {
        newtio.c_cflag &= ~CRTSCTS;
    }
#endif

    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
        return ERR_TC_SET_ATTR;
    }

    return 0;
}

