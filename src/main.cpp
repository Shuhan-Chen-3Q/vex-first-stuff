#include "main.h"
#include "api.h"
#include <iostream>
#include <algorithm>
#include "pros/adi.hpp"
#include "pros/misc.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/*void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text( 2s, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
*/
/**ss
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "GG PROS user");

	//pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
pros::Motor Rightfront(20);
pros::Motor Leftfront(-19);
pros::Motor Rightback(13);
pros::Motor Leftback(-1);
pros::Motor Rightup(11);
pros::Motor Leftup(-18);
pros::Motor Intake(4);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::Pneumatics pistons_ex('a', false);
pros::adi::Pneumatics pistons_re('e', false);
pros::IMU imu(6);
double degToRad(double n) {
	return n * 3.1415 / 180.0;
}

double radToDeg(double n) {
	return n * 180.0 / 3.1415;
}

double rescale360(float angle){
	while(angle > 360){
		angle -= 360;
	}
	return angle; 
}
double rescale180(double n, bool inRad) {
	n = inRad ? radToDeg(n) : n;
	n = n - 360.0 * std::floor((n + 180.0) * (1.0 / 360.0));
	if (inRad) { n = degToRad(n); }
	return n;
}
void moveRobot(float Desired_distance){
	float error = 4567034;
	Rightfront.tare_position();
	Rightback.tare_position();
	Leftfront.tare_position();
	Leftback.tare_position();
	Rightfront.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	Leftfront.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	Rightback.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	Leftback.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	imu.reset();
	pros::delay(3000);
	imu.set_heading(0);
		
	while (true) {
	
		float position = (Rightfront.get_position() + Rightback.get_position() + Leftback.get_position() + Leftfront.get_position()) / 4.0;
		float pi = 3.1415926535897932384626;
		float rotationininch = 4 * pi;
		float distance = rotationininch * position * 200.0/200.0;
		error = Desired_distance - distance;
		float kp = 3;
		float speed = kp * error;

		float current_angle = rescale180(imu.get_heading(), false);
		float angular_error = 0 - current_angle;
		float constant = 1;
		float angular_speed = constant * angular_error;
		float total_left_speed = angular_speed + speed;
		float total_right_speed = -angular_speed + speed;

		

		Rightfront.move(total_right_speed);
		Leftfront.move(total_left_speed);
		Rightback.move(total_right_speed);
		Leftback.move(total_left_speed);

		pros::delay(10);
	}
}



void turnAngle(float desired_angle){
	imu.reset();
	pros::delay(2000);
	float error = 298379;
	while (fabs(error) > 0.1){
		float angle = imu.get_heading();
		error = desired_angle - angle;
		float kp = 2;
		float speed = error * kp;

		Rightback.move(-speed);
		Rightfront.move(-speed);
		Leftback.move(speed);
		Leftfront.move(speed);
		pros::delay(20);
	}
	Rightback.move(0);
	Rightfront.move(0);
	Leftback.move(0);
	Leftfront.move(0);
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void SimplemoveRobot(float Desired_Time){
	Rightfront.move(127);
	Leftfront.move(127);
	Rightback.move(127);
	Leftback.move(127);
	Rightup.move(127);
	Leftup.move(127);
	pros::delay(Desired_Time);
	Rightfront.brake();
	Leftfront.brake();
	Rightback.brake();
	Leftback.brake();
	Rightup.brake();
	Leftup.brake();
}

void autonomous() {
	turnAngle(45);
    pros::delay(10);
    moveRobot(35);
    pros::delay(15);
    turnAngle(50);
    pros::delay(10);
    moveRobot(45);
    pros::delay(10);
    moveRobot(-20);
    pros::delay(10);
    turnAngle(140);

}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


//Hi guys


//bool state = LOW;
//pros:: pneumatic_A = pros::ADIDigitalOut('A', state);
//we are cooked
 /*degToRad(double n) {
	returnADIDigitalOut n * 3.1415 / 180.0;
}
double radToDeg(double n) {
	return n * 180.0 / 3.1415;
}
double rescale180(double n, bool inRad) {
	n = inRad ? radToDeg(n) : n;
	n = n - 360.0 * std::floor((n + 180.0) * (1.0 / 360.0));

	if (inRad) { n = degToRad(n); }
	return n;
}*/




void opcontrol() {
	while (true) {
		float linearSpeed = master.get_analog(ANALOG_LEFT_Y);
		float angularSpeed = master.get_analog(ANALOG_RIGHT_X);
		float total_speed_left = linearSpeed + angularSpeed;
		float total_speed_right = linearSpeed - angularSpeed;

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			Intake.move(-127   );
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			Intake.move(127);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			Intake.brake();
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      		pistons_ex.extend();
			pistons_re.extend();
    	}
    	else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      		pistons_ex.retract();
			pistons_re.retract();
    	}

		/*if (master.get_digital(DIGITAL_L1)){ // moving up
		pneumatic_A.set_value();
		// code to make pneumatics extend
		}
		else if (master.get_digital(DIGITAL_L2)){ // moving down
		pneumatic_A.set_value(LOW);
		// code to make pneumatics retract??
		}*/
	

		Rightfront.move(total_speed_right * 127);
		Leftfront.move(total_speed_left * 127);
		Rightback.move(total_speed_right * 127);
		Leftback.move(total_speed_left * 127);
		Rightup.move(total_speed_right * 127);
		Leftup.move(total_speed_left * 127);
 




		pros::delay(20);                    
	}
	



}




//
 
