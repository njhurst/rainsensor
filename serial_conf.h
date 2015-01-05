#ifndef _SERIAL_CONF_H
#define _SERIAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#define ERR_BAUD -1
#define ERR_BITS -2
#define ERR_PARITY -3
#define ERR_STOP_BITS -4
#define ERR_TC_SET_ATTR -5
#define ERR_TC_GET_ATTR -6

int setPortOptions(int fd, int baudrate, int databits, int parity, int stop,
        bool softwareHandshake, bool hardwareHandshake);

#ifdef __cplusplus
}
#endif

#endif
