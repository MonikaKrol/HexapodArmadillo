//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "TCP/tcpacceptor.h"
#include "Robot/robotcontroler.h"

using namespace std;
//using namespace cv;


int main()
{
    char key = 'm';

    Point3f robotPosition(0, 17, 100);
    Point3f robotAngles(0,0,0);
    float robotWidth = 11.8;
    float robotLength = 36.5;
    Point3f robotLegLenghts(3.7, 5.8, 16.3);

    float walkStep = 5;
    float rotStep = 0.3;
    float sMoveStep = 1;
    float sRotStep = 0.05;

    int delayShort = 10;
    float stepHeight = 4;

    RobotControler rob(walkStep, rotStep, sMoveStep, sRotStep, delayShort, stepHeight, robotPosition, robotAngles, robotWidth, robotLength, robotLegLenghts);

    ///Tryby:
    ///1 - stanie w miejscu i ruch translacyjny
    ///2 - stanie w miejscu i obroty
    ///3 - poruszanie sie manualne
    ///4 - poruszanie sie automatyczne
    ///5 - automatyczne z ruchem nogi po paraboli
    ///6 - automatyczne bez wracania do pozycji poczatkowej
    ///8 - chodzenie do punktu
    ///9 - tryb pokazowy

    int mode = 6;

    TCPStream* stream = NULL;
    TCPAcceptor* acceptor = NULL;
    acceptor = new TCPAcceptor(8080);

    if (acceptor->start() == 0) 
    {
        while(1)
        {
	        stream = acceptor->accept();
	        if (stream != NULL) 
	        { 
	            while(key != 27)
	            {
	                rob.control(key);
	                if (stream->receive(&key, sizeof(key)) > 0) 
	                {
	                    if(key == '8')
                        {
                            char p[2];
                            stream->receive(p, sizeof(p));
                            int x = p[2];
                            x <<= 8;
                            x |= p[1];

                            stream->receive(p, sizeof(p));
                            int y = p[2];
                            y <<= 8;
                            y |= p[1];

                            cout << x << ' ' << y << endl;

                            rob.walkToPoint(Point(x,y));
                        }
	                }
	            }
	        }
            delete stream;
        }
    }
    return 0;
}
