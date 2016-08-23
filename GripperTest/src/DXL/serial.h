
#ifndef _SERIAL_H_
#define _SERIAL_H_

int open_port(const char *pathname);
void set_port_options(int fd);
int close_port(int fd);
int check_port_for_data(int fd, unsigned int sec, unsigned int usec);


#endif

