// 2025-2026 Pushback
#include "main.h"
#include "lemlib/api.hpp"
bool intake1 = false;
bool intake2 = false;
bool pne1 = false;
bool pne2 = false;
bool pne3 = false;
bool intake1_forward = false;  // New variable for forward toggle
bool intake1_reverse = false;  // New variable for reverse toggle

// declare all motor groups and other motors
pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller
pros::MotorGroup left_motor_group({-1, 2, -3}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup right_motor_group({4, -5, 6}, pros::MotorGearset::blue); // right motor group
pros::MotorGroup intake_motor({13}, pros::MotorGearset::blue); // intake motor (flywheels)
pros::MotorGroup intake_motor2({12}, pros::MotorGearset::green); // intake motor #2 (belt)

// declare all pneumatics
pros::ADIDigitalOut pneA('A'); // short piston
pros::ADIDigitalOut pneB('B'); // cat ears (on x button)
pros::ADIDigitalOut pneC('F'); // long piston

// delcare IMU
pros::Imu imu(11); 

// delcare rotational sensors
pros::Rotation vertical_rot_wheel(7); // vertical tracking wheel
pros::Rotation horizontal_rot_wheel(8); // horizontal tracking wheel, reversed

// tracking wheels
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot_wheel, lemlib::Omniwheel::NEW_275, -2.5); // measure the actual tracking wheel offset (TODO)
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot_wheel, lemlib::Omniwheel::NEW_275, -5.75); // measure the actual tracking wheel offset (TODO)


// create the drivetrain
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
	&right_motor_group, // right motor group
	11.5, // 11.5 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
	360, // drivetrain rpm is 360
	2 // horizontal drift is 2 (for now)
);

// create the odometry sensors
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	&horizontal_tracking_wheel, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
	&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors // odometry sensors
);

void initialize() {
	pros::lcd::initialize();

	// reverse the rotational sensors (test and see if needed)
	//vertical_rot_wheel.set_reversed(true); 
	//horizontal_rot_wheel.set_reversed(true); 
	
	chassis.calibrate();
	chassis.setPose(0, 0, 0);

	left_motor_group.tare_position_all();
    right_motor_group.tare_position_all();
    intake_motor.tare_position_all();
	intake_motor2.tare_position_all();

	// print position to the brain
	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
            pros::delay(50);
        }
    });
}

// disabled function for when robot is disabled (field management system or vex comp)
void disabled() {}

// competition initialize function for when robot is connected to field management (can be used for autonomous selectors, etc)
void competition_initialize() {}

// auton function for when we are in autonomous mode
void autonomous() {
	chassis.setPose(-136.887, 21.484, 64.5);
	intake_motor.move_voltage(12000);
	chassis.moveToPoint(-58.5, 58.79, 4000);
	pros::delay(4000);
	chassis.turnToHeading(136.1, 3000);
	pros::delay(2000);
	chassis.moveToPoint(-26.855, 27.29, 3000);
	pros::delay(2000);
	intake_motor.move_voltage(0);
	pros::delay(1000);
	intake_motor.move_voltage(-12000);
	pros::delay(3000);
	chassis.moveToPoint(-40.065, 40.645, 4000, {.forwards = false});
	pros::delay(2000);
	intake_motor.move_voltage(0);
	chassis.turnToHeading(33.5, 3000);
	chassis.moveToPoint(-8.71, 88.113, 4000);
	pros::delay(2000);
	chassis.turnToHeading(0, 3000);
	pros::delay(2000);
	intake_motor.move_voltage(12000);
	chassis.moveToPoint(-8.274, 110.613, 3000);
	pros::delay(2000);
	chassis.moveToPoint(-8.71, 88.113, 3000, {.forwards = false});
	pros::delay(2000);
	intake_motor.move_voltage(0);
	chassis.turnToHeading(270, 3000);
	pros::delay(2000);
	chassis.moveToPoint(-103.477, 88.113, 4000);
	pros::delay(2000);
	chassis.turnToHeading(0, 3000);
	pros::delay(2000);
	chassis.moveToPoint(-103.477, 119.961, 4000);
	pros::delay(2000);
	chassis.turnToHeading(90, 3000);
	pros::delay(2000);
	
	pne1 = !pne1;
	pneA.set_value(pne1);
	pne2 = !pne2;
	pneB.set_value(pne2);

	pros::delay(500);
	
	chassis.moveToPoint(-66.265, 119.961, 4000);
	pros::delay(2000);
	intake_motor2.move_voltage(7500);
	pros::delay(3000);
	intake_motor2.move_voltage(-7500);
	pros::delay(500);
	intake_motor2.move_voltage(0);
	pros::delay(200);
}

// opcontrol function: when we are manually controlling the bot
void opcontrol() {
	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			chassis.setPose(0, 0, 0);
			// tuning pid
			chassis.turnToHeading(90, 10000);
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake1_forward = !intake1_forward;  // Toggle forward state
			intake1_reverse = false;             // Turn off reverse if it's on
			if (intake1_forward) {
				intake_motor.move_voltage(12000);
			} else {
				intake_motor.move_voltage(0);
			}
			pros::delay(200);  // Debounce delay
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake1_reverse = !intake1_reverse;  // Toggle reverse state
			intake1_forward = false;             // Turn off forward if it's on
			if (intake1_reverse) {
				intake_motor.move_voltage(-12000);
			} else {
				intake_motor.move_voltage(0);
			}
			pros::delay(200);  // Debounce delay
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake2 = !intake2;
			if (intake2) {
				intake_motor2.move_voltage(7500);
			} else {
				intake_motor2.move_voltage(0);
			}
			pros::delay(100);
		} 
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake2 = !intake2;
			if (intake2) {
				intake_motor2.move_voltage(-7500);
			} else {
				intake_motor2.move_voltage(0);
			}
			pros::delay(100);
		} 
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			pne1 = !pne1;
			pneA.set_value(pne1);
		} 
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			pne2 = !pne2;
			pneB.set_value(pne2);
		} 
		pros::delay(100);

		
		// joystick values
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		// move the chassis (arcade controls) using lemlibs
		chassis.arcade(leftY, rightX);

		// small delay
		pros::delay(20);
	}
}