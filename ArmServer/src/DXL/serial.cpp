#include <linux/serial.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

/*     The code written up here is based on the examples provided by
 * Michael R. Sweet - "Serial Programming Guide for POSIX Operating Systems"
 *       For more detailed info please visit the website below
 * 
 *        http://www.easysw.com/~mike/serial/serial.html
 *        http://source.winehq.org/source/dlls/ntdll/serial.c
 */


/*
 * Opens the device port
 * pathname: the path to the device file
 */
int open_port(const char *pathname){

  int fd;    /* File descriptor for the port */

  /* Open the port for read and write, don't want to be the controlling terminal
   * of the port and Don't wait for CDS signal  */
  fd = open( pathname , O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1){

      /* PORT could not be opened */
    printf("open_port:Failed to open Port: %s error = %d:%s\n",pathname,errno,strerror(errno));
    return 0;
  }
  else
    /* Set the flow so if there is no data, the read function 
     * returns 0. That is no waiting for data to come */
    fcntl(fd, F_SETFL , FNDELAY);
  
  return (fd);
}




/* Set the port options. This needs to be changed if the
 * user needs a differenct setting 
 */
void set_port_options(int fd){

  struct termios port_options;

  /* Get the port options from the device */
  tcgetattr(fd ,&port_options);

  /* Set 8 bit no parity 1 stop sign (8N1) */
  port_options.c_cflag &= ~PARENB; /* No parity */
  port_options.c_cflag &= ~CSTOPB; /* 1 Stop bit */
  port_options.c_cflag &= ~CSIZE;  
  port_options.c_cflag |= CS8;     /* 8 bit char size */

  /* Other options */
  port_options.c_cflag     |= (CLOCAL | CREAD); /* Enable to read the port, and do not make owner of the port */
  port_options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG); /* Raw output, do not process the output */
  port_options.c_oflag     &= ~OPOST; /* Raw output, do not process the output */
  port_options.c_iflag     |= IGNBRK; /* Ignore the break condition */

  /* Timeout options */
  port_options.c_cc[VMIN] = 0; /* No minimum characters*/
  port_options.c_cc[VTIME]= 0; /* No wait time */

  /* Set the io baud rate */
  cfsetispeed(&port_options , B57600); 
  cfsetospeed(&port_options , B57600);

  tcflush(fd , TCIOFLUSH); /* Flush the input and output buffers */

  /* Set the port options TCSANOW=> set it now without any delay */
  tcsetattr(fd, TCSANOW, &port_options);
}

int close_port(int fd){
     
  int ret;
  ret = close(fd);
     
  if (ret == -1)
    fputs("fd close() failed !! \n",stderr);	 

  return 0;
 }


/* Uses the select statement and waits for the data on the port */
int check_port_for_data(int fd, unsigned int sec, unsigned int usec){    
  
  int            n;
  int            max_fd;
  fd_set         input;
  struct timeval timeout;
  
  /* Initialize the input set */
  FD_ZERO(&input);
  FD_SET(fd, &input);
  
  
  max_fd = fd + 1;

  /* Initialize the timeout structure */
  timeout.tv_sec  = sec;
  timeout.tv_usec = usec;
  
  /* Do the select */
  n = select(max_fd, &input, NULL, NULL, &timeout);
  
  /* See if there was an error */
  if (n < 0)
    perror("check_port_for_data: Select statement failed");
  else if (n == 0){
    puts("check_port_for_data: Select statement timed out  ");
    return 0;	
  }
  else{   
      /* We have input */
      if (FD_ISSET(fd, &input))
	return n;
    }   
  return 0;
}

