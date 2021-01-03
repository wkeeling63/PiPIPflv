#include "GPSUtil.h"
 
int open_gps(GPS_T *gps)
{
   struct termios options, ops;
   memset(&options, 0, sizeof(options));
   options.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
   options.c_iflag = IGNPAR | ICRNL;
   options.c_lflag = ICANON;
   options.c_cc[VEOF]     = 4;     // Ctrl-d 
   options.c_cc[VMIN]     = 1; 
    
   gps->fd_data = open(GPSDATA, O_RDWR | O_NOCTTY ); 
   if (gps->fd_data <0) 
      {
      fprintf(stderr, "Open of GPS data failed! RC=%d\n", gps->fd_data);
      return(gps->fd_data);
      }
   tcgetattr(gps->fd_data,&ops);
   tcflush(gps->fd_data, TCIFLUSH);
   tcsetattr(gps->fd_data,TCSANOW,&options); 
   
   memset(&options, 0, sizeof(options));
   options.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
   options.c_iflag = IGNPAR | ICRNL;
   options.c_lflag = ICANON;
   options.c_cc[VEOF]     = 4;     // Ctrl-d 
   options.c_cc[VMIN]     = 1; 
      
   gps->fd_cntl = open(GPSCNTL, O_RDWR | O_NOCTTY ); 
   if (gps->fd_cntl <0) 
      {
      fprintf(stderr, "Open of GPS control failed! RC=%d\n", gps->fd_cntl);
      return(gps->fd_cntl);
      }
   tcflush(gps->fd_cntl, TCIFLUSH);
   tcsetattr(gps->fd_cntl,TCSANOW,&options); 
   
   write(gps->fd_cntl, "AT+QGPS=1\r", 10);
   
   return 0;
}

int close_gps(GPS_T *gps)
{
   int status=0;
   write(gps->fd_cntl, "AT+QGPSEND\r", 11);
     
   status=close(gps->fd_cntl);
   if (status)
      {
      fprintf(stderr, "Close of GPS control failed! RC=%d\n", status);
      } 
   status=close(gps->fd_data);
   if (status)
      {
      fprintf(stderr, "Close of GPS data failed! RC=%d\n", status);
      }
   
   return status;
}
void read_gps(GPS_T *gps)
{
   int cnt, i, c=0;
   int index[20];
   char buf[255];
   cnt = read(gps->fd_data,buf,255);
   cnt--;
   buf[cnt]=0;
   if ((cnt) && (!(strncmp(buf,"$GPRMC",6))))
      {
      index[0]=0;
      for (i=0;i<cnt;i++)
         {
          if (buf[i]==',')
            {
            buf[i]=0;
            c++;
            index[c]=i+1;
            }
         }
         if (buf[index[2]]== 'A')
            {
            int spd=0;
            sscanf(buf+index[7], "%d", &spd);
            gps->speed=spd*1.15078;   
            }
         else
            {
            gps->speed=0;
            }
      }
      
   return;
}

cairo_surface_t* cairo_text(char *text)
{
   int width = 96, height = 32;
   cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
   cairo_t *cr =  cairo_create(surface);
   cairo_rectangle(cr, 0, 0, width, height);
   cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.0);
   cairo_fill(cr);
   cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
   cairo_select_font_face(cr, "cairo:serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
   cairo_set_font_size(cr, 18);
   cairo_move_to(cr, 0.0, 20.0);
   cairo_text_extents_t extents;
   cairo_text_extents(cr, text, &extents);
   cairo_show_text(cr, text);
   return(surface);
}
