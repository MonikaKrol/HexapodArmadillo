#ifndef MAESTRO_H
#define MAESTRO_H

#include <termios.h>

class Maestro
{
    private:
        const char * device;  // Linux
        int fd;
        struct termios options;
        
    public:
        Maestro();
        ~Maestro();

        int getError();
        int getPosition(unsigned char channel);
        int setTarget(unsigned char channel, unsigned short target);
};

#endif // MAESTRO_H
