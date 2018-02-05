#ifndef ROBOTWALK_H
#define ROBOTWALK_H

//#include <opencv2/core/core.hpp>
#include "src/Robot/robot.h"
#include "src/Robot/Point.h"

class RobotWalk
{
	private:

		int delayLong; //delay between each step in function walk
        int delayShort; //delay used between each iteration walk walkStraight and walkStraightAlt
        bool firstStep;

        int walkingSequnceNo;
        int rotatingSequenceNo;

        //steps used in rotating to prevent calculating each rotation every time
        //used in simpleRotation
        Point3f stepsl[6];

        float smallMotion; // small motions leg will do in smooth walking 

        float stepHeight;
	
	public:

		RobotWalk(int delayShort1 = 10, float stepHeight1 = 4);

		//simple walk with setting leg end to position in the middle
		//each call to function provides different step of sequence ( 5 walkingSequnceNo)
		void simpleWalk(Point3f step, Robot& rob);

		//simple walk with automatic sequence calling with given delayLong
	    void simpleAutomaticWalk(Point3f step, Robot& rob);

	    //smooth steps with parabolic move only straight
	    void walkStraight(float step, Robot& rob);

	    //walkStraight but legs 042 start sequence and then 315 start
	    void walkStraightAlt(float step, Robot& rob);

	    //modes 0 - start 1 - walking 2 - end
	    void walkStepAhead(float step, int mode, Robot& rob);

	    //smooth steps in any direction
	    void walk(Point3f step, Robot& rob);

	    //instead of walking robot rotates by given angle
	    //each call to function provides different step of sequence ( 5 rotatingSequenceNo)
	    void simpleRotation(float angle, Robot& rob);

	    //simpleRotation with automatic sequence calling with given delayLong
	    void simpleAutomaticRotation(float angle, Robot& rob);

	    //rotation with smooth parabolic motion
	    void rotation(float angle, Robot& rob);
};

#endif