#ifndef ROBOT_H
#define ROBOT_H

//#include <opencv2/core/core.hpp>
#include <armadillo>
#include "src/util.h"
#include "src/Robot/leg.h"
#include "src/maestro.h"
#include "src/Robot/Point.h"

class Robot
{
    private:
        rect lFrame, gFrame;// l - lokalne g - globalne

        //position and angles of middles point in robots base
        Point3f position;
        Point3f angles;

        //initial values ( will be set in case of restart )
        Point3f initPosition;
        Point3f initAngles;
        
        //width and length of robots base
        float width, length;

        arma::fmat Rx, Ry, Rz, R;
        Leg legs[6];

        Maestro device;

        void moveCoordinates(Point3f p, Point3f ang);

    public:
        Robot(Point3f pos, Point3f ang, float width1, float length1, Point3f leglengths);

        Point3f getPosition(){return position;};
        Point3f getAngles(){return angles;};
        rect getFrame();
        joints getLegJoints(int n);
        Leg getLeg(int n) const {return legs[n];}

        void restart(Point3f pos, Point3f ang); // restart robot to its base position

        void move(Point3f p); // moves robot base ( only base moves - leg ends stays the same )
        void rotate(Point3f ang); // rotates robot base ( only base rotates - leg ends stays the same )

        void moveLeg(int n, Point3f p);
};

#endif // ROBOT_H
