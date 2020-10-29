#include <stdio.h>
#include <stdlib.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <string.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <webots/compass.h>
#include <webots/camera_recognition_object.h>

#define TIME_STEP 32
#define GRIPPER_MOTOR_MAX_SPEED 0.1

////////////////Main funtion

int main() {

	int i, k;
  //i:to count the quantity of objects recognized, k is to judge the type of objects
           int a=0;
            
           int b=0; 
  //a, b to assist-- to slow down the speed and avoid turning over
           int key;

	int seed=1;
	
	int food=1;   //the sign of throwing fish food to avoid detect orange box again

	int finish=0;
	
	int width, height;

	int red_l, blue_l, green_l, red_r, blue_r, green_r, red_s, blue_s, green_s;

	///////////////////initialization

	wb_robot_init();
	/* Get the gripper device, enable it*/
	WbDeviceTag gripper_motors0 = wb_robot_get_device("lift motor");
	WbDeviceTag gripper_motors1 = wb_robot_get_device("left finger motor");
	WbDeviceTag gripper_motors2 = wb_robot_get_device("right finger motor");

	/* Get the camera device, enable it, and store its width and height */
	WbDeviceTag camera_right = wb_robot_get_device("camera_right");
	wb_camera_enable(camera_right, TIME_STEP);

	WbDeviceTag camera_side = wb_robot_get_device("camera_side");
	wb_camera_enable(camera_side, TIME_STEP);

	WbDeviceTag camera_left = wb_robot_get_device("camera_left");
	wb_camera_enable(camera_left, TIME_STEP);

	width = wb_camera_get_width(camera_side);
	height = wb_camera_get_height(camera_side);
	
	WbDeviceTag camera = wb_robot_get_device("camera");
	wb_camera_enable(camera, TIME_STEP);

	wb_camera_recognition_enable(camera, TIME_STEP);

	WbDeviceTag compass = wb_robot_get_device("compass");
	wb_compass_enable(compass, TIME_STEP);

	/* get a handler to the motors and set target position to infinity (speed control). */
	WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
	WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
	WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
	WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

	wb_motor_set_position(front_left_wheel, INFINITY);
	wb_motor_set_position(front_right_wheel, INFINITY);
	wb_motor_set_position(back_left_wheel, INFINITY);
	wb_motor_set_position(back_right_wheel, INFINITY);

	double left_speed = 0.0, right_speed = 0.0;
	wb_motor_set_velocity(front_left_wheel, left_speed);
	wb_motor_set_velocity(front_right_wheel, right_speed);
	wb_motor_set_velocity(back_left_wheel, left_speed);
	wb_motor_set_velocity(back_right_wheel, right_speed);


	bool avoid_tree = 0;//avoid the trees while turning
	bool cross_bridge = 0;//turn and cross the bridge
	bool find_tree = 0;//tree detected
	bool find_bridge = 0;//bridge detected
	bool find_arch = 0;//arch detected
	bool cross_arch = 0;//turn and cross the arch

	void step(double seconds) {
		const double ms = seconds * 1000.0;
		int elapsed_time = 0;
		while (elapsed_time < ms) {
			wb_robot_step(TIME_STEP);
			elapsed_time += TIME_STEP;
		}
	}

	void lift(double position) {
		wb_motor_set_velocity(gripper_motors0, GRIPPER_MOTOR_MAX_SPEED);
		wb_motor_set_position(gripper_motors0, position);
	}

	void moveFingers(double position) {
		wb_motor_set_velocity(gripper_motors1, GRIPPER_MOTOR_MAX_SPEED);
		wb_motor_set_velocity(gripper_motors2, GRIPPER_MOTOR_MAX_SPEED);
		wb_motor_set_position(gripper_motors1, position);
		wb_motor_set_position(gripper_motors2, position);
	}

	void moveForwards(double speed) {
		wb_motor_set_velocity(front_left_wheel, speed);
		wb_motor_set_velocity(front_right_wheel, speed);
		wb_motor_set_velocity(back_left_wheel, speed);
		wb_motor_set_velocity(back_right_wheel, speed);
	}
           void turn(double speed) {
                      wb_motor_set_velocity(front_left_wheel, speed);
                      wb_motor_set_velocity(front_right_wheel, -speed);
                      wb_motor_set_velocity(back_left_wheel, speed);
                      wb_motor_set_velocity(back_right_wheel, -speed);
           }
	void stop(double seconds) {
		wb_motor_set_velocity(front_left_wheel, 0.0);
		wb_motor_set_velocity(front_right_wheel, 0.0);
		wb_motor_set_velocity(back_left_wheel, 0.0);
		wb_motor_set_velocity(back_right_wheel, 0.0);
		step(seconds);
	}
///////////////////////////////////////////////The main function of releasing fish food
	int releasing_food() {
		  stop(0.5);
		  moveFingers(0.02);
                        step(0.5);
                        turn(2.0);
                        step(2.0);
                        moveForwards(1.3);
                        step(2.5);
                        stop(0.5);
                        moveFingers(0.06);
                        step(0.5);
                        stop(1.0);
                        moveForwards(-1.3);
                        step(2.5);
                        stop(0.5);
                        turn(-2.1);
                        step(2.0);
                        stop(0.5);
                        moveFingers(0.04);
                        step(0.5);
                        lift(0.02);
                        step(0.5);
                        food=0;
		return 0;
	}


///////////////////////////////////////////the function of object recognition
	int object_recognition() {
           int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
	//printf("\nRecognized %d objects.\n", number_of_objects);
           const WbCameraRecognitionObject* objects = wb_camera_recognition_get_objects(camera);	
           for (i = 0; i < number_of_objects; ++i) {
			//printf("Model of object %d: %s\n", i, objects[i].model);
			k = *(objects[i].model);
			if ((k == 112) || (k == 115)) {//tree detected
				find_tree = 1;
			}
			if (k == 111) {//bridge detected
				find_bridge = 1;
			}
			if (k == 65) {//arch detected
				find_arch = 1;
			}
			//printf("%d\n", k);
		}
		// read distance sensors
		const double* north = wb_compass_get_values(compass);
		double angle = atan2(north[0], north[2]);
		double bearing = (angle - 1.5708) / M_PI * 180.0;
		if (bearing < 0.0) {
			bearing += 360.0;
		}
		//printf("bearing is %f\n", bearing);//the current angle of heading forward 

		bool South = (bearing >= 266) && (bearing <= 269.6);
		bool North = (bearing >= 86) && (bearing <= 89.6);
		bool East = (bearing >= 176) && (bearing <= 179.6);
		bool West = ((bearing >= 356) && (bearing <= 360)) || ((bearing >= 0) && (bearing <= 1.6));
		//the angle of east, west, south and north


		if ((avoid_tree == 0) && (cross_bridge == 0) && (cross_arch==0)&&(food==0)&&(finish==0)) {//simply move forward
			left_speed = 6.4, right_speed = 6.4;
			left_speed = 6.4, right_speed = 6.4;

			wb_motor_set_velocity(front_left_wheel, left_speed);
			wb_motor_set_velocity(front_right_wheel, right_speed);
			wb_motor_set_velocity(back_left_wheel, left_speed);
			wb_motor_set_velocity(back_right_wheel, right_speed);
		}

		if (find_tree) {
			avoid_tree = 1;
		}

		if (find_arch) {
			cross_arch = 1;
			finish=1;
		}

		if (find_bridge) {
			cross_bridge = 1;
			a = 1;
		}

		//to get the time on bridge by a, b
		if (a == 1) {
			b++;
		}

		// turn right to avoid the tree
		if ((West == 0) && avoid_tree) {
			left_speed = 2, right_speed = -2;
			left_speed = 2, right_speed = -2;

			wb_motor_set_velocity(front_left_wheel, left_speed);
			wb_motor_set_velocity(front_right_wheel, right_speed);
			wb_motor_set_velocity(back_left_wheel, left_speed);
			wb_motor_set_velocity(back_right_wheel, right_speed);
		}

		//turn left to cross the bridge
		if ((South == 0) && cross_bridge) {
			left_speed = -2, right_speed = 2;
			left_speed = -2, right_speed = 2;

			wb_motor_set_velocity(front_left_wheel, left_speed);
			wb_motor_set_velocity(front_right_wheel, right_speed);
			wb_motor_set_velocity(back_left_wheel, left_speed);
			wb_motor_set_velocity(back_right_wheel, right_speed);
		}
		//turn left to cross the arch		
		if ((South == 0) && cross_arch) {
			left_speed = -2, right_speed = 2;
			left_speed = -2, right_speed = 2;

			wb_motor_set_velocity(front_left_wheel, left_speed);
			wb_motor_set_velocity(front_right_wheel, right_speed);
			wb_motor_set_velocity(back_left_wheel, left_speed);
			wb_motor_set_velocity(back_right_wheel, right_speed);
		}
		//slow down when going downward on bridge
		if ((b >= 92) && (b <= 120)) {
			left_speed = 3, right_speed = 3;
			left_speed = 3, right_speed = 3;

			wb_motor_set_velocity(front_left_wheel, left_speed);
			wb_motor_set_velocity(front_right_wheel, right_speed);
			wb_motor_set_velocity(back_left_wheel, left_speed);
			wb_motor_set_velocity(back_right_wheel, right_speed);
		}


		//printf("south is %d\n", South);
		//printf("north is %d\n", North);
		//printf("west is %d\n", West);
		//printf("east is %d\n", East);
		//printf("find_bridge is %d\n", find_bridge);
		//printf("cross_bridge is %d\n", cross_bridge);
		//printf("find_tree is %d\n", find_tree);
		//printf("avoid_tree is %d\n", avoid_tree);
		//printf("a is %d\n", a);
		//printf("b is %d\n", b);

		//when the actions are all done, the criteria is set to 0

		if (West) {
			avoid_tree = 0;
			find_tree = 0;
		}
		if (South) {
			cross_bridge = 0;
			find_bridge = 0;
			find_arch = 0;
			cross_arch=0;
		}

		
	return 0;
           } 

///////////////////////////////////////////////////////the funtion of tracking line
          int tracking(){
	const unsigned char* image_r = wb_camera_get_image(camera_right);
	const unsigned char* image_l = wb_camera_get_image(camera_left);
	const unsigned char* image_m = wb_camera_get_image(camera_side);
	/* Reset the sums */
	red_l = 0;
	green_l = 0;
	blue_l = 0;

	red_r = 0;
	green_r = 0;
	blue_r = 0;

	red_s = 0;
	green_s = 0;
	blue_s = 0;

	
		
	/*
	  * Here we analyse the image from the three cameras.for tracking The goal is to detect a
	* (a spot of color) of a defined color in the sidedle of our
	* screen.*/

	for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++) {
			red_r = wb_camera_image_get_red(image_r, width, x, y);
			green_r = wb_camera_image_get_green(image_r, width, x, y);
			blue_r = wb_camera_image_get_blue(image_r, width, x, y);

			red_l = wb_camera_image_get_red(image_l, width, x, y);
			green_l = wb_camera_image_get_green(image_l, width, x, y);
			blue_l = wb_camera_image_get_blue(image_l, width, x, y);
		}


	if (seed) {
		image_m = wb_camera_get_image(camera_side);

		for (int x = 0; x < width; x++)
			for (int y = 0; y < height; y++) {
				red_s = wb_camera_image_get_red(image_m, width, x, y);
				green_s = wb_camera_image_get_green(image_m, width, x, y);
				blue_s = wb_camera_image_get_blue(image_m, width, x, y);
			}


              if ((red_s> 3 * green_s) && (red_s> 3 * blue_s)){
              key='0';//red
              seed=0;}
          
			 
              else if ((red_s> 3 * green_s) &&(blue_s> 3 * green_s)){
              key='1';//purple
              seed=0;}
				
				
              else if ((red_s> 3 * blue_s) &&(green_s> 3 * blue_s)){
              key='2';//yellow
              seed=0;}
          
              else if((red_s> 1.5 * green_s) && (red_s> 3 * blue_s)&&(green_s> 1.5 * blue_s)){
              //orange box is detected, throw the fish food
              key='3';}
	
              else{ 
              key='0';
              seed=1;}

	}

          if((red_l> 3 * green_l) && (red_l> 3 * blue_l)&&(red_r> 3 * green_r) && (red_r> 3 * blue_r)){
          //red box at the end is detected, stop
          key='4';}
          
	///////////////////choose speed
	switch (key) {
	case'0':
		if (red_r - red_l > 30) {
			left_speed = 6.4, right_speed = 6.4-0.02*(red_r-red_l);
		}
		else if (red_l - red_r > 30) {
			left_speed = 6.4-0.02*(red_l-red_r), right_speed = 6.4;
		}
		else {
			left_speed = right_speed = 6.4;
		}
		break;

	case '1':
		if (blue_r - blue_l > 30) {
			left_speed = 6.4, right_speed = 6.4-0.02*(blue_r-blue_l);
		}
		else if (blue_l - blue_r > 30) {
			left_speed = 6.4-0.02*(blue_l-blue_r), right_speed = 6.4;
		}
		else {
			left_speed = right_speed = 6.4;
		}
		break;

	case '2':
		if (green_r - green_l > 30) {
			left_speed = 6.4, right_speed = 6.4-0.02*(green_r-green_l);
		}
		else if (green_l - green_r > 30) {
			left_speed = 6.4-0.02*(green_l-green_r), right_speed = 6.4;
		}
		else {
			left_speed = right_speed = 6.4;
		}
		break;

	case '3':
              	if(food==1){
		releasing_food();}
		break;
	
	case '4':
	           left_speed =0.0; right_speed = 0.0;


	}
	///////////////////change speed
	wb_motor_set_velocity(front_left_wheel, left_speed);
	wb_motor_set_velocity(front_right_wheel, right_speed);
	wb_motor_set_velocity(back_left_wheel, left_speed);
	wb_motor_set_velocity(back_right_wheel, right_speed);
	return 0;}


/* Main loop --------------------------------------------------------------------------------------------------------------------------------*/
while (wb_robot_step(TIME_STEP) != -1) {
	tracking();
	object_recognition();
}

wb_robot_cleanup();

return 0;
}