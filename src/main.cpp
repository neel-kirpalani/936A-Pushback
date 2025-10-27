// 2025-2026 Pushback
#include "main.h"
#include "lemlib/api.hpp"
#include "liblvgl/lvgl.h"

// ===== GLOBAL VARIABLES =====
bool intake1 = false;
bool intake2 = false;
bool pne1 = false;
bool pne2 = false;
bool pne3 = false;
bool intake1_forward = false;
bool intake1_reverse = false;

// Auton selector variable
int selected_auton = 1; // 1 = Left Red, 2 = Right Red, 3 = Left Blue, 4 = Right Blue, 5 = Skills

// ===== MOTOR AND SENSOR DECLARATIONS =====
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motor_group({-1, 2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({4, -5, 6}, pros::MotorGearset::blue);
pros::MotorGroup intake_motor({13}, pros::MotorGearset::blue);
pros::MotorGroup intake_motor2({12}, pros::MotorGearset::blue);

pros::ADIDigitalOut pneA('A');
pros::ADIDigitalOut pneB('B');
pros::ADIDigitalOut pneC('F');

pros::Imu imu(11);

//pros::Rotation vertical_rot_wheel(7);
//pros::Rotation horizontal_rot_wheel(8);

//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot_wheel, lemlib::Omniwheel::NEW_275, -2.5);
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot_wheel, lemlib::Omniwheel::NEW_275, -5.75);

lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 11.5, lemlib::Omniwheel::NEW_325, 400, 2);
//lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);


lemlib::ControllerSettings lateral_controller(15, 0, 6, 0, 0, 0, 0, 0, 0);



lemlib::ControllerSettings angular_controller(3.2, 0, 17, 0, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// ========== LVGL AUTON SELECTOR (v8/v9 API) ==========
// Button event callback for LVGL v8/v9
static void auton_btn_event_cb(lv_event_t * e) {
    lv_obj_t* btn = lv_event_get_target_obj(e);
    int id = (int)(intptr_t)lv_obj_get_user_data(btn);
    selected_auton = id;

    // Show selection on info label
    static lv_obj_t* info_label = lv_obj_get_child(lv_screen_active(), 5); // label is 6th child (5-based index)
    if (info_label) {
        const char* auton_names[] = {
            "", "Left Red", "Right Red", "Left Blue", "Right Blue", "Skills"
        };
        lv_label_set_text_fmt(info_label, "Selected: %s", auton_names[selected_auton]);
    }
}

// Set up auton selector GUI on brain screen
void auton_selector_create() {
    // Title
    lv_obj_t* title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "Select Autonomous:");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    // Button data
    const char* btn_titles[] = {"Left Red", "Right Red", "Left Blue", "Right Blue", "Skills"};
    int btn_ids[] = {1, 2, 3, 4, 5};
    for (int i = 0; i < 5; i++) {
        lv_obj_t* btn = lv_button_create(lv_screen_active());
        lv_obj_set_size(btn, 90, 40);
        lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 10 + (95*i), 40);
        lv_obj_set_user_data(btn, (lv_obj_user_data_t)(intptr_t)btn_ids[i]);
        lv_obj_add_event_cb(btn, auton_btn_event_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label, btn_titles[i]);
        lv_obj_center(label);
    }

    // Info label
    lv_obj_t* info_label = lv_label_create(lv_screen_active());
    lv_label_set_text(info_label, "Selected: Left Red");
    lv_obj_align(info_label, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ===== INITIALIZE FUNCTION =====
void initialize() {
    pros::lcd::initialize();

    chassis.calibrate();
    chassis.setPose(0, 0, 0);

    left_motor_group.tare_position_all();
    right_motor_group.tare_position_all();
    intake_motor.tare_position_all();
    intake_motor2.tare_position_all();

    // Add auton selector using modern LVGL API
    auton_selector_create();

    // Position display task
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(5, "X: %f", chassis.getPose().x);
            pros::lcd::print(6, "Y: %f", chassis.getPose().y);
            pros::lcd::print(7, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
}

// ===== DISABLED FUNCTION =====
void disabled() {
    // this code runs when the bot is connected to field control and is disabled
    // Test autonomous
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        autonomous();
    }
    // Test opcontrol
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        opcontrol();
    }
}

// ===== COMPETITION INITIALIZE =====
void competition_initialize() {
    // Selector is already created in initialize()
    // This runs after initialize() when connected to field control
}

// ===== AUTONOMOUS FUNCTION =====
void autonomous() {
    if (selected_auton == 1) {
        // LEFT RED AUTON
        chassis.setPose(-46.392, 9.509, 90); // initial set pos
        intake_motor.move_voltage(12000); // start the intake (flywheels)
        chassis.moveToPoint(-31.197, 9.509, 1500, {.maxSpeed=35}, false); // 1st point
        pros::delay(500); 
        chassis.turnToHeading(31.2, 2000, {}, false); // turn to face 2nd point
        pros::delay(500);
        chassis.moveToPoint(-23.031, 23.146, 1200, {.maxSpeed=80}, false); // 2nd point
        pros::delay(500); // delay to intake the blocks
        chassis.turnToHeading(136.1, 3000); // turn to face 3rd point
        pros::delay(300);
        chassis.moveToPoint(-13.984, 13.745, 3000, {.maxSpeed = 50}); // 3rd point
        pros::delay(1000);
        intake_motor.move_voltage(-12000); // outtake blocks
        pros::delay(1200);
        chassis.moveToPoint(-15.774, 16.002, 4000, {.forwards = false, .maxSpeed = 40}); // move backwards to 4th point
        pros::delay(1000);
        intake_motor.move_voltage(0); // stop the intake
        chassis.turnToHeading(34.7, 2000); // turn to 5th point
        chassis.moveToPoint(-3.429, 34.69, 3000, {.maxSpeed = 80}); // move to 5th point
        pros::delay(1000);
        chassis.turnToHeading(0, 3000); // turn to 6th point
        pros::delay(500);
        intake_motor.move_voltage(12000); // start the intake
        chassis.moveToPoint(-3.257, 43.548, 2000, {.maxSpeed = 50}, false); // move to 6th point
        pros::delay(1300);
        chassis.moveToPoint(-3.429, 34.69, 2000, {.forwards = false, .maxSpeed = 50}, false); // move backwards to 7th point
        pros::delay(500);
        intake_motor.move_voltage(0); // stop the intake
        chassis.turnToHeading(270, 3000); // turn to face the 8th point
        pros::delay(2000);
        chassis.moveToPoint(-40.739, 34.69, 3000, {.maxSpeed=80}, false); // move to the 8th point
        pros::delay(1000);
        chassis.turnToHeading(0, 3000); // turn to face 9th point
        pros::delay(2000);
        chassis.moveToPoint(-40.739, 47.229, 2000, {.maxSpeed = 40}, false); // move to 9th point
        pros::delay(2000);
        chassis.turnToHeading(90, 1500, {}, false); // turn to face the 10th point
        pros::delay(500);
        intake_motor.move_voltage(10000); // start intake to push blocks up to allow the barrel to lift up (flywheels)
        // activate both pistons to lift up barrel
        pne1 = !pne1; 
        pneA.set_value(pne1);
        pne2 = !pne2;
        pneB.set_value(pne2);

        pros::delay(1000);
        intake_motor.move_voltage(0); // stop the flywheels
    
        chassis.moveToPoint(-26.089, 47.229, 3000, {.maxSpeed = 20}, false); // move to the 10th point
        pros::delay(1000);
        intake_motor2.move_voltage(10000); // push blocks out into goal
        pros::delay(3000); // 3 seconds to push all the blocks out
        intake_motor2.move_voltage(-10000); // bring chain back to initial position
        pros::delay(500);
        intake_motor2.move_voltage(0); // stop intake motor (chain)
        pros::delay(200);
    }
    else if (selected_auton == 2) {
        // RIGHT RED AUTON
        chassis.setPose(-47.497, -4.821, 90); // initial set pos
        chassis.moveToPoint(-31.32,-4.821,3000,{.maxSpeed=50}, false); // move to 1st point
        pros::delay(1500);
        chassis.turnToHeading(152.3, 1500); // turn to face 2nd point
        intake_motor.move_voltage(10000); // run intake flywheels
        pros::delay(500);
        chassis.moveToPoint(-22.118, -22.349, 3000, {.maxSpeed=30}, false); // move to 2nd point
        pros::delay(1500);
        chassis.turnToHeading(44.8, 1500); // turn to face 3rd point
        pros::delay(500);
        chassis.moveToPoint(-14.686, -14.866, 2000, {.maxSpeed = 30}, false); // move to 3rd point
        pros::delay(500);
        intake_motor.move_voltage(-10000); // start outaking (flywheels)
        pros::delay(2000);
        chassis.moveToPoint(-22.118, -22.349, 2000, {.forwards=false,.maxSpeed= 30}); // move to 4th point (moving backwards)
        pros::delay(500);
        intake_motor.move_voltage(0); // stop intake (flywheels)
        chassis.turnToHeading(135, 1500); // turn to face 5th point
        pros::delay(500);
        chassis.moveToPoint(-4.393, -32.27, 1500); // move to 5th point
        pros::delay(500);
        chassis.turnToHeading(180, 1500); // turn to face 6th point
        intake_motor.move_voltage(10000);
        pros::delay(500);
        chassis.moveToPoint(-4.393, -40.793, 2000, {.maxSpeed=45}); // move to 6th point
        pros::delay(1500);
        chassis.moveToPoint(-4.393, -32.27, 1000); // move to 7th point (move backwards)
        pros::delay(500);
        chassis.turnToHeading(270, 1000); // turn to 8th point
        intake_motor.move_voltage(0);
        pros::delay(500);
        chassis.moveToPoint(-36.442, -32.27, 2000, {.maxSpeed=80}, false); // move to 8th point
        pros::delay(500);
        chassis.turnToHeading(180, 1000); // turn to face 9th point
        pros::delay(500);
        chassis.moveToPoint(-36.442, -47.482, 1000, {.maxSpeed=50}); // move to 9th point
        pros::delay(500);
        chassis.turnToHeading(90, 1000); // turn to face goal (10th point)
        pros::delay(500);
        
        // lift up barrel
        pne1 = !pne1; 
		pneA.set_value(pne1);
		pne2 = !pne2;
		pneB.set_value(pne2);

        pros::delay(1500);
        chassis.moveToPoint(-31.32, -47.482, 1000, {.maxSpeed=20});
        pros::delay(500);
        intake_motor2.move_voltage(10000); // start pushing blocks out of barrel
        pros::delay(1500);
        intake_motor2.move_voltage(-10000); // move chain back to initial pos
        pros::delay(500);
        intake_motor2.move_voltage(0);
        pros::delay(200);
        
    }
    else if (selected_auton == 3) {
        // LEFT BLUE AUTON
        chassis.setPose(0,0,0);
        chassis.turnToHeading(180, 10000);
    }
    else if (selected_auton == 4) {
        // RIGHT BLUE AUTON
        // ...Your code here
    }
    else if (selected_auton == 5) {
        // SKILLS AUTON
        // ...Your code here
    }
}

// ===== OPCONTROL FUNCTION =====
void opcontrol() {
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            // LEFT RED AUTON
            chassis.setPose(-46.392, 9.509, 90); // initial set pos
            intake_motor.move_voltage(12000); // start the intake (flywheels)
            chassis.moveToPoint(-31.197, 9.509, 1500, {.maxSpeed=35}, false); // 1st point
            pros::delay(500); 
            chassis.turnToHeading(31.2, 2000, {}, false); // turn to face 2nd point
            pros::delay(500);
            chassis.moveToPoint(-23.031, 23.146, 1200, {.maxSpeed=80}, false); // 2nd point
            pros::delay(500); // delay to intake the blocks
            chassis.turnToHeading(136.1, 3000); // turn to face 3rd point
            pros::delay(300);
            chassis.moveToPoint(-13.984, 13.745, 3000, {.maxSpeed = 50}); // 3rd point
            pros::delay(1000);
            intake_motor.move_voltage(-12000); // outtake blocks
            pros::delay(1200);
            chassis.moveToPoint(-15.774, 16.002, 4000, {.forwards = false, .maxSpeed = 40}); // move backwards to 4th point
            pros::delay(1000);
            intake_motor.move_voltage(0); // stop the intake
            chassis.turnToHeading(34.7, 2000); // turn to 5th point
            chassis.moveToPoint(-3.429, 34.69, 3000, {.maxSpeed = 80}); // move to 5th point
            pros::delay(1000);
            chassis.turnToHeading(0, 3000); // turn to 6th point
            pros::delay(500);
            intake_motor.move_voltage(12000); // start the intake
            chassis.moveToPoint(-3.257, 43.548, 2000, {.maxSpeed = 50}, false); // move to 6th point
            pros::delay(1300);
            chassis.moveToPoint(-3.429, 34.69, 2000, {.forwards = false, .maxSpeed = 50}, false); // move backwards to 7th point
            pros::delay(500);
            intake_motor.move_voltage(0); // stop the intake
            chassis.turnToHeading(270, 3000); // turn to face the 8th point
            pros::delay(2000);
            chassis.moveToPoint(-40.739, 34.69, 3000, {.maxSpeed=80}, false); // move to the 8th point
            pros::delay(1000);
            chassis.turnToHeading(0, 3000); // turn to face 9th point
            pros::delay(2000);
            chassis.moveToPoint(-40.739, 47.229, 2000, {.maxSpeed = 40}, false); // move to 9th point
            pros::delay(2000);
            chassis.turnToHeading(90, 1500, {}, false); // turn to face the 10th point
            pros::delay(500);
            intake_motor.move_voltage(10000); // start intake to push blocks up to allow the barrel to lift up (flywheels)
            // activate both pistons to lift up barrel
            pne1 = !pne1; 
            pneA.set_value(pne1);
            pne2 = !pne2;
            pneB.set_value(pne2);

            pros::delay(1000);
            intake_motor.move_voltage(0); // stop the flywheels
        
            chassis.moveToPoint(-26.089, 47.229, 3000, {.maxSpeed = 20}, false); // move to the 10th point
            pros::delay(1000);
            intake_motor2.move_voltage(10000); // push blocks out into goal
            pros::delay(3000); // 3 seconds to push all the blocks out
            intake_motor2.move_voltage(-10000); // bring chain back to initial position
            pros::delay(500);
            intake_motor2.move_voltage(0); // stop intake motor (chain)
            pros::delay(200);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            // put code in here for button B
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake1_forward = !intake1_forward;
            intake1_reverse = false;
            if (intake1_forward) {
                intake_motor.move_voltage(12000);
            } else {
                intake_motor.move_voltage(0);
            }
            pros::delay(200);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake1_reverse = !intake1_reverse;
            intake1_forward = false;
            if (intake1_reverse) {
                intake_motor.move_voltage(-12000);
            } else {
                intake_motor.move_voltage(0);
            }
            pros::delay(200);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake2 = !intake2;
            if (intake2) {
                intake_motor2.move_voltage(8000);
            } else {
                intake_motor2.move_voltage(0);
            }
            pros::delay(100);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake2 = !intake2;
            if (intake2) {
                intake_motor2.move_voltage(-8000);
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

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        pros::delay(20);
    }
}
