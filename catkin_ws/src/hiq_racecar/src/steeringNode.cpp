#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "../include/hiq_racecar/JHPWMPCA9685.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// Channels
#define SERVO_CHANNEL 0
#define MOTOR_CHANNEL 1

// Servo and motor constants
#define FREQUENCY 60
#define IN_MIN -100
#define IN_MAX 100
#define MOTOR_MAX_BACKWARDS 249 //249
#define MOTOR_MAX_FORWARDS 399 //512
#define MOTOR_MIN_FORWARDS 399 //399
#define SERVO_RIGHT 261 //261
#define SERVO_LEFT 520 //520 (524 was a bit too much)
#define SERVO_STRAIGHT (SERVO_LEFT + SERVO_RIGHT) / 2

// ROS constants
#define LOOP_RATE 10

struct ValuesNormalized {
    int servoNormalized;
    int motorNormalized;
};

class SteeringNode {

	int currentServoOutput, currentMotorOutput, currentServoNormalized, currentMotorNormalized;

	int getkey() {
		int character;
		struct termios orig_term_attr;
		struct termios new_term_attr;

		/* set the terminal to raw mode */
		tcgetattr(fileno(stdin), &orig_term_attr);
		memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
		new_term_attr.c_lflag &= ~(ECHO|ICANON);
		new_term_attr.c_cc[VTIME] = 0;
		new_term_attr.c_cc[VMIN] = 0;
		tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

		/* read a character from the stdin stream without blocking */
		/*   returns EOF (-1) if no character is available */
		character = fgetc(stdin);

		/* restore the original terminal attributes */
		tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

		return character;
	}

	int map ( int x, int outMin, int outMax) {
		int toReturn =  (x - IN_MIN) * (outMax - outMin) / (IN_MAX - IN_MIN) + outMin ;
		return toReturn ;
	}

	ValuesNormalized parseToNormalizedValues(const std_msgs::String::ConstPtr& msg) {
                std::string msgString = *msg;
		std::string delimiter = ",";
		std::string servoNormalized = msg.substr(0, msg.find(delimiter));
		std::string motorNormalized = msg.substr(1, msg.find(delimiter));

		ValuesNormalized valuesNormalized = {.servoNormalized = servoNormalized,
											 .motorNormalized = motorNormalized};

		return valuesNormalized;
	}

	public:

	void updateSteering(const std_msgs::String::ConstPtr& msg) {
		//Debug?
		ROS_INFO("I heard: [%s]", msg->data.c_str());

		int servoNormalized, motorNormalized, servoOutput, motorOutput;
		ValuesNormalized valuesNormalized;

		// Parse to struct with normalized values
		valuesNormalized = parseToNormalizedValues(msg);
		servoNormalized = valuesNormalized.servoNormalized;
		motorNormalized = valuesNormalized.motorNormalized;
		
		// Update servo output if changed
		if ( servoNormalized != currentServoNormalized ) {

			servoOutput = map(servoNormalized, SERVO_LEFT, SERVO_RIGHT);
			//pca9685->setPWM(SERVO_CHANNEL, 0, servoOutput);
			ROS_INFO("Setting servo output to: %s", servoOutput);

		}

		// Update motor output if changed
		if ( motorNormalized != currentMotorNormalized ) {

			motorOutput = map(motorNormalized, MOTOR_MAX_BACKWARDS, MOTOR_MAX_FORWARDS);
			//pca9685->setPWM(MOTOR_CHANNEL, 0, motorOutput);
			ROS_INFO("Setting motor output to: %s", motorOutput);

		}
	}
	
	SteeringNode() {

		currentServoOutput = 0;
		currentMotorOutput = 0;	

	}

	int getCurrentServoOutput;

}; // end SteeringNode

int main(int argc, char **argv) {

	PCA9685 *pca9685 = new PCA9685();

	int err = pca9685->openPCA9685();
	if (err < 0){

	    printf("Error: %d", pca9685->error);

	} else {

		printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress);
	    pca9685->setAllPWM(0,0);
	    pca9685->reset();
	    pca9685->setPWMFrequency(FREQUENCY);

		SteeringNode *steeringNode = new SteeringNode();		

		// Initialize ROS stuff
		ros::init(argc, argv, "steering_node");
		ros::NodeHandle steering_node;
		ros::Subscriber sub = steering_node.subscribe("steering_values", 1000, SteeringNode::updateSteering);
		ros::Rate loop_rate(LOOP_RATE);

	    while(pca9685->error >= 0 && ros::ok()){

			// TODO: Do something to listen to the topic
			ros::spinOnce();
		    loop_rate.sleep();

	    }
	}
	pca9685->closePCA9685();
	return 0;
}
