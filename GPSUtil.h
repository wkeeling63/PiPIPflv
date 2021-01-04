
#ifndef GPSUTIL_H_
#define GPSUTIL_H_

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cairo/cairo.h>
#define BAUDRATE B115200            
#define GPSDATA "/dev/ttyUSB1"
#define GPSCNTL "/dev/ttyUSB2"
// TEXTW must be 32 aligned
#define TEXTW 256
// TEXTH must be 16 aligned
#define TEXTH 64

typedef struct GPS_S 
   {
   int fd_data;
   int fd_cntl;
   int speed;
   int active;
   } GPS_T;
   
int open_gps(GPS_T *gps);
int close_gps(GPS_T *gps);
void read_gps(GPS_T *gps);
cairo_surface_t* cairo_text(int, int, int);

#endif /* GPSUTIL_H_ */
