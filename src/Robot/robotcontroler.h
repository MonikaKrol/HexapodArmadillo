#ifndef ROBOTCONTROLER_H
#define ROBOTCONTROLER_H

#include "src/Robot/robot.h"
#include "src/Robot/robotwalk.h"
#include "src/Robot/point.h"

class RobotControler
{
	private:
		Robot rob;
		RobotWalk walker;

		float walkStep; // length of step which robot will take
		float rotStep; // angle of rotation of robot

		float sMoveStep; // how much will robot move its base
		float sRotStep; // how much will robot rotate its base

		Point3f defaultRobotPosition; //coordinates to which robot will be restarted
		Point3f defaultRobotAngles; //angles to which robot will be restarted

		int mode;

		bool startedStepAhead;
		int directionStepAhead;

		int directionContinuous;

	public:
		RobotControler(float walkStep1, float rotStep1, float sMoveStep1, float sRotStep1, int delayShort, float stepHeight, Point3f pos, Point3f ang, float width1, float length1, Point3f leglengths);

		Robot& getRobot() {return rob;};

		void simpleWalking(char direction);

		void simpleAutomaticWalking(char direction);

		void smoothWalking(char direction);

		void walkingStepAhead(char direction, int stage);

		void continuousWalking(char direction);

		///directions
		/// 0 - forward
		/// 1 - backward
		/// 2 - left
		/// 3 - right
		/// 4 - up
		/// 5 - down
		void moveBase(char direction); // move only robots base without moving legends
		void rotateBase(char direction); // rotate only robots base without moving legends

		void restart() {rob.restart(defaultRobotPosition, defaultRobotAngles);}; //resets robot position

		void walkToPoint(Point2f point);
        void showoff();

        void control(char key);
};


#endif