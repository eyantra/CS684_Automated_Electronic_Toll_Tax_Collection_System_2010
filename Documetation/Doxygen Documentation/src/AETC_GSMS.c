/**
 * @file AETC_GSMS.c
 * Program for serial communication between Owner and TollPlaza through GSM Module.
 * Send Success Message
 * @author Puskar Kothavade, Ashish Pardhi, Mugdha Nazare, IIT Bombay
 * @date 10/Oct/2010
 * @version 1.0
 *
 * @section LICENSE
  * Copyright (c) 2010. ERTS Lab IIT Bombay
   * All rights reserved.

  * Redistribution and use in source and binary forms, with or without
   *modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
    * notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
   * notice, this list of conditions and the following disclaimer in
   * the documentation and/or other materials provided with the distribution.

   * Neither the name of the copyright holders nor the names of
   * contributors may be used to endorse or promote products derived
   * from this software without specific prior written permission.

   * Source code can be used for academic purpose.
   * For commercial use permission form the author needs to be taken.
 */
#include<stdio.h>
#include<string.h>
#include<malloc.h>
#include<unistd.h>
#include<fcntl.h>
#include<errno.h>
#include<termios.h>
#include <sys/time.h>
#include <sys/types.h>


/**
* Open port for serial communication.
* display error messsage if unable to open.
*/
int port_open(void)
{
 int fd;
  fd = open("/dev/ttyS0",O_RDWR | O_NOCTTY | O_NDELAY);
  if(fd == -1)
  {
    perror("Unable to open the port: /dev/ttyS0");
  }
  else
  { 
    fcntl(fd,F_SETFL,0);
  }
  return(fd); //return file descriptor
}

/**
* Configure port
* @param fd File Descriptor to access serial port
*/

void port_config(int fd)
{
  struct termios settings;
  
  tcgetattr(fd,&settings);
  
  cfsetispeed(&settings,9600);
  cfsetospeed(&settings,9600);

  settings.c_cflag |= (CLOCAL | CREAD);

  settings.c_cflag &= ~PARENB;
  settings.c_cflag &= ~CSTOPB;
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |=  CS8;

  tcsetattr(fd,TCSANOW,&settings);
}

/**
*Generate Delay of two seconds.
*
*/

void del_2s(void)      // Delay for wait
{
  unsigned char r, s;
  for(r=0;r<125;r++)
    for(s=0;s<255;s++);

   void del_1s(void)
  {
   unsigned char p, q;
   for(p=0;p<125;p++)
     for(q=0;q=125;q++);
  }
}


/**
*Send Message to register owner
* @param fd File Descriptor to access serial port
* @param *c character pointer 
*/
void write_data(int fd,char *c)
{
  char s[2] = "";
  int n;
  char *charPointer;

  charPointer = (char *)malloc(sizeof(char) * 10);//Allocate memory

  sprintf(charPointer,"%x",26);

  n = write(fd, c, strlen(c));
  if(n<0)
  {
    fputs("write() of data failed! \n",stderr);
  }
  printf("sent String: %d\n",n);
  
   n = write(fd, "\nAT\r", 4);
   sleep(2);
 
   n = write(fd, "AT+IPR=9600\r",12);
   sleep(2);

   n = write(fd, "AT+CSQ\r", 7);
   sleep(2);
  
   n = write(fd, "AT+CMGF=1\r", 10);
   sleep(2);
  
   n = write(fd, "AT+CMGS=\"9619821002\"\r", 21);
   sleep(5);
   n = write(fd, "Dear Customer, 100 Rupees Have Been Deducted From Your Account. Thank You!\n", 76);
   n = write(fd, "\x1a", 3);	
   sleep(2);

   n = write(fd, "\rAT+CMGR=1\n", 11);

}

/**
* Close port for reuse
* @param fd File Descriptor to access serial port
*/
void port_close(int fd)
{
  close(fd);
}

int main(void)
{ 
  int fd;///< File Descriptor to access serial port
  fd = port_open(); // Open Port
  port_config(fd); //Configure Port
  write_data(fd,""); //Write Message
  del_2s(); //Generate Delay
  port_close(fd); //Close Port
  return 0; //Return
}


