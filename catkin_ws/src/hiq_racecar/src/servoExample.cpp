#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <JHPWMPCA9685.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int servoFullBackwards = 249 ;//249
int servoFullForwards = 399 ;//512
int servoMinForwards = 408;
int servoStandStill = 380;//(servoFullForwards + servoFullBackwards) / 2;
int servoRight = 261 ; //261
int servoLeft = 520 ;//524 was a bit too much
int servoStraight = (servoRight + servoLeft) / 2;

int servoChannel = 0;
int motorChannel = 1;

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

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoFullBackwards, servoFullForwards)
// Maps 90 degrees to the servo value

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "subscribe_and_publish");
    ros::NodeHandle n;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    int count = 0;

    PCA9685 *pca9685 = new PCA9685();

    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0);
        pca9685->reset();
        pca9685->setPWMFrequency(60) ;
        while(pca9685->error >= 0 && ros::ok()){

	    // Make a circle
            pca9685->setPWM(servoChannel,0,servoRight) ;
            pca9685->setPWM(motorChannel,0,servoMinForwards) ;

        }
    }
    pca9685->closePCA9685();
}
