#include "main.h"
#include "lemlib/api.hpp"
bool intake1 = false;
bool intake2 = false;

// declare all motor groups and other motors
pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller
pros::MotorGroup left_motor_group({-1, 2, -3}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup right_motor_group({4, -5, 6}, pros::MotorGearset::blue); // right motor group
pros::MotorGroup intake_motor({13}, pros::MotorGearset::blue); // intake motor (flywheels)
pros::MotorGroup intake_motor2({12}, pros::MotorGearset::green); // intake motor #2 (belt)

// delcare IMU
pros::Imu imu(10); 

// delcare rotational sensors
pros::Rotation vertical_rot_wheel(7); // vertical tracking wheel
pros::Rotation horizontal_rot_wheel(8); // horizontal tracking wheel, reversed

// tracking wheels
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot_wheel, lemlib::Omniwheel::NEW_275, -2.5); // measure the actual tracking wheel offset (TODO)
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot_wheel, lemlib::Omniwheel::NEW_275, -5.75); // measure the actual tracking wheel offset (TODO)


// create the drivetrain
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
	&right_motor_group, // right motor group
	10, // 10 inch track width
	lemlib::Omniwheel::NEW_4, // using new 4" omnis
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
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
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
            
            pros::delay(20);
        }
    });
}

// disabled function for when robot is disabled (field management system or vex comp)
void disabled() {}

// competition initialize function for when robot is connected to field management (can be used for autonomous selectors, etc)
void competition_initialize() {}

// auton function for when we are in autonomous mode
void autonomous() {
	// tuning pid
	chassis.turnToHeading(90, 10000);
}

// opcontrol function: when we are manually controlling the bot
void opcontrol() {
	while (true) {
		// joystick values
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		// move the chassis (arcade controls) using lemlibs
		chassis.arcade(leftY, rightX);

		// small delay
		pros::delay(20);
	}
}