#include "src/Robot/robotcontroler.h"
#include <iostream>
#include <chrono>
#include <thread>
//#include <opencv2/highgui/highgui.hpp>

using namespace std::this_thread;
using namespace std::chrono;
//using namespace cv;
using namespace std;

RobotControler::RobotControler(float walkStep1, float rotStep1, float sMoveStep1, float sRotStep1, int delayShort, float stepHeight, Point3f pos, Point3f ang, float width1, float length1, Point3f leglengths)
 : rob(pos, ang, width1, length1, leglengths), walker(delayShort, stepHeight)
{
	walkStep = walkStep1;
	rotStep = rotStep1;

	sMoveStep = sMoveStep1;
	sRotStep = sRotStep1;

	defaultRobotPosition = pos;
	defaultRobotAngles = ang;

	mode = 6;

    startedStepAhead = false;
    directionStepAhead = 0;

    directionContinuous = 0;
}

void RobotControler::control(char key)
{
	switch(key)
    {
        case '1':
            mode = 1;
            break;
        case '2':
            mode = 2;
            break;
        case '3':
            mode = 3;
            break;
        case '4':
            mode = 4;
            break;
        case '5':
            mode = 5;
            break;
        case '6':
            mode = 6;
            break;
        case '7':
        	mode = 7;
        	break;
        case '8':
        	mode = 8;
        	break;
        case '9':
        	mode = 9;
        	break;
        case 'r':
        	restart();
        	break;
    }

    switch(mode)
    {
        case 1:
            moveBase(key);
            break;
        case 2:
            rotateBase(key);
            break;
        case 3:
            simpleWalking(key);
            break;
        case 4:
            simpleAutomaticWalking(key);
            break;
        case 5:
            smoothWalking(key);
            break;
        case 6:
            if(!startedStepAhead)
            {
                walkingStepAhead(key, 0);
            }
            else
                walkingStepAhead(key, 1);

            break;
        case 7:
            continuousWalking(key);
            break;
        case 8:
        
            break;
        case 9:
            showoff();
            mode = 5;
            break;
    }

    if(startedStepAhead && (mode != 6))
    {
        if(directionStepAhead == 1)
            walkingStepAhead('w', 2);
        else if (directionStepAhead == 2)
            walkingStepAhead('s', 2);
        startedStepAhead = false;
    }
}

void RobotControler::walkingStepAhead(char direction, int stage)
{
    switch(direction)
    {
        case 'd':
            walker.walk(Point3f(walkStep, 0, 0), rob);
            break;
        case 'a':
            walker.walk(Point3f(-walkStep, 0, 0), rob);
            break;
        case 'w':
            
            walker.walkStepAhead(walkStep, stage, rob);

            directionStepAhead = 1;
            
            startedStepAhead = true;
            break;
        case 's':
            
            walker.walkStepAhead(-walkStep, stage, rob);

            directionStepAhead = 2;
            startedStepAhead = true;
            break;
        case 'e':
            walker.rotation(rotStep, rob);
            break;
        case 'q':
            walker.rotation(-rotStep, rob);
            break;
        default:
            return;
    }
}

void RobotControler::continuousWalking(char direction)
{
    if(directionContinuous == 0)
    {
        switch(direction)
        {
            case 'w':
                walker.walkStepAhead(walkStep, 0, rob);
                directionContinuous = 1;
                break;
            case 's':
                walker.walkStepAhead(-walkStep, 0, rob);
                directionContinuous = 2;
                break;
        }
    }
    else
    {
        if(directionContinuous == 1)
            walker.walkStepAhead(walkStep, 1, rob);
        else if(directionContinuous == 2)
            walker.walkStepAhead(-walkStep, 1, rob);

        if(direction == 'z')
        {
            if(directionContinuous == 1)
                walker.walkStepAhead(walkStep, 2, rob);
            else if(directionContinuous == 2)
                walker.walkStepAhead(-walkStep, 2, rob);

            directionContinuous = 0;
        }
    }
}

void RobotControler::simpleWalking(char direction)
{
    switch(direction)
    {
        case 'd':
            walker.simpleWalk(Point3f(walkStep, 0, 0), rob);
            break;
        case 'a':
            walker.simpleWalk(Point3f(-walkStep, 0, 0), rob);
            break;
        case 'w':
            walker.simpleWalk(Point3f(0, 0, walkStep), rob);
            break;
        case 's':
            walker.simpleWalk(Point3f(0, 0, -walkStep), rob);
            break;
        case 'e':
            walker.simpleRotation(rotStep, rob);
            break;
        case 'q':
            walker.simpleRotation(-rotStep, rob);
            break;
        default:
            return;
    }
}

void RobotControler::simpleAutomaticWalking(char direction)
{
    switch(direction)
    {
        case 'd':
            walker.simpleAutomaticWalk(Point3f(walkStep, 0, 0), rob);
            break;
        case 'a':
            walker.simpleAutomaticWalk(Point3f(-walkStep, 0, 0), rob);
            break;
        case 'w':
            walker.simpleAutomaticWalk(Point3f(0, 0, walkStep), rob);
            break;
        case 's':
            walker.simpleAutomaticWalk(Point3f(0, 0, -walkStep), rob);
            break;
        case 'e':
            walker.simpleAutomaticRotation(rotStep, rob);
            break;
        case 'q':
            walker.simpleAutomaticRotation(-rotStep, rob);
            break;
        default:
            return;
    }
}

void RobotControler::smoothWalking(char direction)
{
    switch(direction)
    {
        case 'd':
            walker.walk(Point3f(walkStep, 0, 0), rob);
            break;
        case 'a':
            walker.walk(Point3f(-walkStep, 0, 0), rob);
            break;
        case 'w':
            walker.walk(Point3f(0, 0, walkStep), rob);
            break;
        case 's':
            walker.walk(Point3f(0, 0, -walkStep), rob);
            break;
        case 'e':
            walker.rotation(rotStep, rob);
            break;
        case 'q':
            walker.rotation(-rotStep, rob);
            break;
        default:
            return;
    }
}

void RobotControler::moveBase(char direction)
{
	Point3f step;
	switch(direction)
	{
		case 'w':
			step = Point3f(sMoveStep,0,0);
			break;
		case 's':
			step = Point3f(-sMoveStep,0,0);
			break;
		case 'a':
			step = Point3f(0,0,sMoveStep);
			break;
		case 'd':
			step = Point3f(0,0,-sMoveStep);
			break;
		case 'q':
			step = Point3f(0,sMoveStep,0);
			break;
		case 'e':
			step = Point3f(0,-sMoveStep,0);
			break;
		default:
			//cout << "Nieprawidłowy kierunek" << endl;
			return;
	}
	rob.move(step);
}

void RobotControler::rotateBase(char direction)
{
	Point3f rotation;
	switch(direction)
	{
		case 'w':
			rotation = Point3f(sRotStep,0,0);
			break;
		case 's':
			rotation = Point3f(-sRotStep,0,0);
			break;
		case 'a':
			rotation = Point3f(0,0,sRotStep);
			break;
		case 'd':
			rotation = Point3f(0,0,-sRotStep);
			break;
		case 'q':
			rotation = Point3f(0,sRotStep,0);
			break;
		case 'e':
			rotation = Point3f(0,-sRotStep,0);
			break;
		default:
			//cout << "Nieprawidłowy kierunek" << endl;
			return;
	}
	rob.rotate(rotation);
}

void RobotControler::walkToPoint(Point2f point)
{
    float destAngle = atan2(point.y, point.x);
    float differenceAngle = destAngle - rob.getAngles().y;

    int rotateTimes = differenceAngle/rotStep;

    for(int i = 0; i < rotateTimes; ++i)
    {
        walker.rotation(rotStep, rob);
    }
    walker.rotation(differenceAngle-rotateTimes*rotStep, rob);

    float distance = sqrt(point.x*point.x + point.y*point.y);

    int walkTimes = distance/walkStep;

    //walker.walkStraightAlt2((distance-walkTimes*walkStep), 0, rob, view1);
    walker.walkStepAhead(walkStep, 0, rob);
    for(int i = 0; i < walkTimes; ++i)
    {
        walker.walkStepAhead(walkStep, 1, rob);
    }
    walker.walkStepAhead(walkStep, 2, rob);
    //walker.walkStraightAlt2((distance-walkTimes*walkStep), 2, rob, view1);
}

void RobotControler::showoff()
{
    int wait = 500;

    rob.restart(Point3f(0, 17 ,100), Point3f(0,0,0));

    rob.move(Point3f(2,0,0));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(-4,0,0));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(2,0,0));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(0,0,2));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(0,0,-4));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(0,0,2));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(0,3,0));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(0,-6,0));
    sleep_for(nanoseconds(wait));

    rob.move(Point3f(0,3,0));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0,0.1,0));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0,-0.2,0));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0,0.1,0));
    sleep_for(nanoseconds(wait));
    
    rob.rotate(Point3f(0.1,0,0));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(-0.2,0,0));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0.1,0,0));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0,0,0.2));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0,0,-0.4));
    sleep_for(nanoseconds(wait));

    rob.rotate(Point3f(0,0,0.2));
    sleep_for(nanoseconds(wait));
}