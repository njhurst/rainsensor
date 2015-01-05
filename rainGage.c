#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "serial_conf.h"
const char *names[] = {"PeakRS", "SPeakRS", "RainAD8", "LRA", "TransRat", "AmbLNoise"};
const char *bits[] = {"PkOverThr", "Raining", "Out1On", "HtrOn", "IsDark", "Cndnstn", "Freeze", "Storm"};
const char *slowRegs[] = {"RevLevel", "EmLevel", "RecEmStr", "ABLevel", "TmprtrF", "PUGain", "ClearTR", "AmbLight", "Bucket", "Barrel", "RGConfig", "DwellT", "SinceRn", "MonoStb", "LightAD", "RainThr"};

int main (int argc, char *argv[]) {
    int baud = 1200, fd_ser = -1, ret = 0;
    const char *devname;

    if (argc < 2 || argc > 3) {
        fprintf(stdout, "Usage: %s serial_port_path [baud]\n", argv[0]);
        return -1;
    }

    devname = argv[1];

    if (argc > 2) {
        baud = atoi(argv[2]);
    }

    // Get a file descriptor, no controlling TTY, read and write permissions
    fd_ser = open(devname, (O_RDWR | O_NOCTTY));

    // Check to see if we actually had permission to do that
    if (fd_ser < 0) {
        fprintf(stderr, "Failed to open port \"%s\"\n", devname);
        return -1;
    }

    // Configure the serial port
    ret = setPortOptions(
            fd_ser,  // pass in the file descriptor
            baud,    // the baud rate
            8,       // data bits
            0,       // parity bits
            1,       // stop bit
            false,   // no software handshake (XON/XOFF)
            false    // no hardware handshake
        );

    if (ret < 0) {
        fprintf(stderr, "Failed to set port options with error: %d\n", ret);
        close(fd_ser);
        return -1;
    }

    uint8_t inBuf[512];
    unsigned int slow_reg_index, slow_reg_val[64] = {0};
    char reg_hold[3];
    ssize_t bytes_read, bytes_buffered, bytes_processed = 0;

    reg_hold[2] = '\0';

    fprintf(stdout, "\e[H\e[J");

    do {
        bytes_buffered = 0;
        while (bytes_buffered < 19) {
            bytes_buffered += bytes_read = read(fd_ser, inBuf + bytes_buffered, (sizeof(inBuf)/sizeof(*inBuf)) - bytes_buffered);
            if (bytes_read <= 0) {
                fprintf(stderr, "Unable to read from port!\n");
                exit(0);
            } 
        } 

        for (int i = 0; i < bytes_buffered; i++) {
            if (bytes_processed == 0 && inBuf[i] == 's') {
                bytes_processed = 1;
                fprintf(stdout, "\e[H");
            } else if (bytes_processed > 0) {
                bytes_processed += 1;

                if (bytes_processed % 2 == 1) {
                    reg_hold[1] = inBuf[i];
                    fprintf(stdout, "\e[K");

                    switch (bytes_processed) {
                        case 3:
                        case 5:
                        case 7:
                        case 9:
                        case 11:
                        case 13:
                            fprintf(stdout, "%s: 0x%s\n", names[(bytes_processed - 3) / 2], reg_hold);
                            break;
                        case 15: {
                            unsigned int reg;
                            sscanf(reg_hold, "%x", &reg);
                            for(int j = 0; j < sizeof(bits)/sizeof(*bits); j++) {
                                if(reg & 1) {
                                    fprintf(stdout, "%s, ", bits[j]);
                                }
                                reg >>= 1;
                            }
                            fprintf(stdout, "\n");
                            break;
                        }
                        case 17:
                            sscanf(reg_hold, "%x", &slow_reg_index);
                            break;
                        case 19:
                            sscanf(reg_hold, "%x", &slow_reg_val[slow_reg_index]);
                            fprintf(stdout, "\t");
                            for (int j = 0; j < sizeof(slowRegs)/sizeof(*slowRegs); j++) {
                                fprintf(stdout, "%s: 0x%02x;\t", slowRegs[j], slow_reg_val[j]);
                                if ((j % 4) == 3) {
                                    fprintf(stdout, "\n\t");
                                }
                            }
                            fprintf(stdout, "\n");
                            bytes_processed = 0;
                            break;
                        default:
                            fprintf(stderr, "AHHHHHHHH!\n");
                            bytes_processed = 0;
                            break;
                    }
                } else {
                    reg_hold[0] = inBuf[i];
                }
            } else {
                fprintf(stderr, "Discarding 0x%02x\n", inBuf[i]);
            }
        }
    } while (1);

    close(fd_ser);

    return 0;
}
