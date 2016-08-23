
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_BITS 8
#define SPI_DEVICE1 "/dev/spidev0.0"
#define SPI_DEVICE2 "/dev/spidev0.1"

void main(int argc,char *argv[]);
int MCP3208(int fd,unsigned int ch,unsigned int speed);
int setupspi(unsigned int speed);
int trxspi(int fd,unsigned int databytes,unsigned char tx[],unsigned char rx[],unsigned int speed);
float get_cputemp(void);
void get_daytime (char Time_string[],char fname[],char timec[]);

int setupspi(unsigned int speed)
{ 
       int fd,ret;
            unsigned char mode,bits;
      fd=open(SPI_DEVICE1,O_RDWR);
  if (fd <0 ) { printf("Cannot OPEN SPI0\n");
             exit(-1);
            }
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret <0 ) { printf("Cannot set Max speed%d(Hz)\n",speed);
       close(fd);  
  exit(-1);}

       mode = SPI_MODE_0;
   ret = ioctl(fd,SPI_IOC_WR_MODE,&mode);
        if (ret <0 ) { printf("Cannot set WRmode %d\n",mode);
      close(fd);
    exit(-1);}

 ret = ioctl(fd,SPI_IOC_RD_MODE,&mode);
       if (ret <0 ) { printf("Cannot set RDmode %d\n",mode);
    close(fd);
      exit(-1);}

       bits = SPI_BITS;
   ret = ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&bits);
        if (ret <0 ) { printf("Cannot set WRbits %d\n",bits);
      close(fd);    
 exit(-1);}
  
    return(fd);
}  

int trxspi(int fd,unsigned int databytes,unsigned char tx[],unsigned char rx[],unsigned int speed)
{   
  
int ret;
struct spi_ioc_transfer tr = 
	{
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = databytes,
		.delay_usecs = 0,
		.speed_hz = speed,
		.bits_per_word = SPI_BITS,
};
 
ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
     printf("Cannot send/receive spi message\n");
       return(-1);
}
      return(databytes);

}

void main(int argc,char *argv[])
{
  FILE *fp;
   int fd;
       unsigned short int value[2],i;
    float volt[2],temp[2],cputemp;
  int speed=1000000;
    char daytime[20];
      char data[32768];
       char fname[30];
       char  afname[150];
         char timec[20];
     char *c;
 get_daytime(daytime,fname,timec);
//        fd=setupspi(speed);
//for (i=0;i<2;i++)value[i]=MCP3208(fd,i,speed);
//   close(fd);

cputemp=get_cputemp();
    sprintf(data," VVV DE %s VVV DE %s CPU %5f TIME %s +",
                   argv[1],argv[1],cputemp,daytime);
//for (i=0;i<2;i++){
//     volt[i]=value[i]*3.3/4095.0;
//       temp[i]=(volt[i]-0.6)*100;
//   }
  //  sprintf(data,"%s %10f %10f %10f \n",timec,cputemp,temp[0],temp[1]);

//sprintf(afname,"%senvlog.txt",argv[1]);
//  c=strstr(timec,"00:00");
//  if (c == NULL ) {fp=fopen(afname,"a+");} else {fp=fopen(afname,"wt"); }
     fp=fopen("send.txt","wt");
      fprintf(fp,"%s",data);
    fclose(fp);


}

int MCP3208(int fd,unsigned int ch,unsigned int speed)
{

unsigned int chl,chh;
      unsigned char tx[3];
              unsigned char rx[3];
  int value,i,ret;
       long ava=0;

       ch=ch<<6;
         chl=ch & 0xff;
          chh=ch>>8;

       for(i=0;i<256;i++){
    tx[0]=0x06 | chh;
            tx[1]=0x3F | chl;
               tx[2]=0xFF;
ret=trxspi(fd,3,tx,rx,speed);
      value=((int)(rx[1])<<8)+(int)rx[2];
  ava+=value;
         }

  ava=ava >> 8;
    return((int)ava);
}

float get_cputemp ()
{  FILE *fd;
     int temp;
      float temp_act;
     fd=fopen ("/sys/class/thermal/thermal_zone0/temp","r");
       fscanf(fd,"%d",&temp);
          temp_act=(float)temp/1000.0;
      //   printf("%f \n",temp_act);
    fclose(fd);
     return(temp_act);
}

void get_daytime (char Time_string[],char fname[],char timec[])
 {
    time_t now;
    struct tm *tm;
    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        printf ("Error extracting time stuff\n");
        return;
    }

    sprintf (Time_string,"%04d%02d%02d %02d%02d",
        tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec);

    sprintf (fname,"%04d-%02d-%02d.dat",
        tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday);

    sprintf (timec,"%02d:%02d",
        tm->tm_hour, tm->tm_min);

    return;
}
