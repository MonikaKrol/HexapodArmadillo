#ifndef LEG_H
#define LEG_H

///klasa nogi z implementacja kinematyki odwrotnej opartej na
///https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
///https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/

#include <armadillo>
//#include <opencv2/core/core.hpp>
#include "src/Robot/Point.h"
#include "src/util.h"
#include "src/maestro.h"

class Leg
{
    private:
        joints legJoints; //joint A - przy podstawie, nastepne po koleji
        Point3f legEnd;
        Point3f angles;//katy na zgieciach x - kat przy polaczeniu A, y przy B, z przy C
        Point3f initAngles;
        Point3f lengths;//dlugosci poszczegolnych czesci nog x-AB y-BC z-CD
        arma::fmat R; //macierz obrotu taka jak robota
        Point3f relAngles; //katy wzgledne

        Point3f signals;
        Point3i servos;
        Maestro* device;
        void calculateJointPoints();
        void calculateServoSignals();
    public:
        Leg(Point3f joint1, Point3f angles1, Point3f lengths1, Point3f signals1);
        Leg(){};

        joints getJoints(){return legJoints;};

        void initJointPoints();

        void setJointA(Point3f joint1) {legJoints.A = joint1;};
        void setAngles(Point3f angles1) {angles = angles1;};
        void setLengths(Point3f lengths1) {lengths = lengths1;};
        void setLegEnd(Point3f legEnd1) {legEnd = legEnd1;};
        void setR(arma::fmat R1) {R = R1;};
        void setSignals(Point3f sig) {signals = sig;};
        void setServos(Point3i servos1) {servos = servos1;};
        void setDevice(Maestro* dev) {device = dev;};

        int calculateAngles();
};

#endif // LEG_H
