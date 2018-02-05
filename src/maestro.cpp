#include "maestro.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>

Maestro::Maestro() : device("/dev/ttyAMA0")
{
    fd = open(device, O_RDWR | O_NOCTTY);

    tcgetattr(fd, &options);
    /*cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);*/
    /*cfsetispeed(&options, B230400);
    cfsetospeed(&options, B230400);*/
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // no flow control
    options.c_cflag &= ~CRTSCTS;

    options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    options.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 20;

    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        perror("init_serialport: Couldn't set term attributes");
        return;
    }

    if (fd == -1)
    {
        perror(device);
    }
}

Maestro::~Maestro()
{
    close(fd);
}

int Maestro::setTarget(unsigned char channel, unsigned short target)
{
    unsigned char command[] = {0xAA, 0xC, 0x04, channel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }
    return 0;
}

int Maestro::getError()
{
    unsigned char command[] = { 0xAA, 0xC, 0x21 };
    if (write(fd, command, sizeof(command)) != 3)
    {
        perror("error writing");
        return -1;
    }

    int n = 0;
    unsigned char response[2];
    do
    {
        int ec = read(fd, response+n, 1);
        if(ec < 0)
        {
            perror("error reading");
            return ec;
        }
        if (ec == 0)
        {
            continue;
        }
        n++;

    } while (n < 2);

    //Helpfull for debugging
    //printf("Error n: %d\n", n);
    //printf("Error secon: %d\n", response[1]);

    return (int)sqrt(response[0] + 256*response[1]);
}

int Maestro::getPosition(unsigned char channel)
{
    unsigned char command[] = {0xAA, 0xC, 0x10, channel};
    if(write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }

    int n = 0;
    char response[2];
    do
    {
        int ec = read(fd, response+n, 1);
        if(ec < 0)
        {
            perror("error reading");
            return ec;
        }
        if (ec == 0)
        {
            continue;
        }
        n++;

    } while (n < 2);

    return response[0] + 256*response[1];
}
