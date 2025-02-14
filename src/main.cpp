#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

// global variables
bool inAuton = true;
bool direction = false;
bool on = false;

// constants for lb PID
const int numStates = 3;
int driverStates[numStates] = {0, 34, 180};
int currState = 0;
int target = 0;

pros::Controller master(pros::E_CONTROLLER_MASTER); 

lemlib::ExpoDriveCurve driveCurve(3, 10, 1.019);

pros::MotorGroup left_mg({-19, -18, -17}, pros::MotorGearset::blue);    
pros::MotorGroup right_mg({12, 13, 14}, pros::MotorGearset::blue); 

pros::Motor flexwheel(20, pros::MotorGearset::green);
pros::Motor chain(11, pros::MotorGearset::blue);
pros::Motor lb(8, pros::MotorGearset::green);

pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false); 

lemlib::Drivetrain drivetrain(
	&left_mg,
    &right_mg,
    11.25,
	lemlib::Omniwheel::NEW_325,
    450, 
    2
);

// sensors
pros::Imu imu(16); 
pros::Distance distance_sensor(9);
pros::Rotation yOdom(15); 
pros::Rotation ladyOdom(21); 
lemlib::TrackingWheel vertical_tracking_wheel(
    &yOdom, 
    lemlib::Omniwheel::NEW_2,
    -2.5
); 


// defining sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, 
    nullptr,
    nullptr,
    nullptr,
    &imu
);

// PID
lemlib::ControllerSettings lateral_controller(
	10, // (kP)
    0, // (kI)
    3, // (kD)
    3, // anti windup
    1, // small error range, in inches
	100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
	2, // (kP)
	0, // (kI)
    10, // (kD)
	3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// chassis definition
lemlib::Chassis chassis(
    drivetrain, 
    lateral_controller,
    angular_controller, 
    sensors, 
    &driveCurve,
    &driveCurve
);

// functions to change the PID/how fast lb moves
void liftControl(){
    double kp;
    if(inAuton){
        kp = 0.5;
    } else{
        kp = 3;
    }
    double error = target - (ladyOdom.get_position()/100.0);
    double velocity = kp * error;
    lb.move(velocity);
}

// get to the next state for lb
void nextState(){
    currState += 1;
    if(currState == 3){
        currState = 0;
    }
    target = driverStates[currState];
}


void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	imu.set_heading(0);

    pros::delay(50);

    ladyOdom.reset_position();
    ladyOdom.reset();

    pros::delay(50);

    yOdom.reset_position(); 

    pros::delay(50);

    left_mg.set_brake_mode(pros::MotorBrake::brake);
    right_mg.set_brake_mode(pros::MotorBrake::brake);

	pros::delay(50);

    chassis.calibrate();
	pros::delay(500);

	pros::lcd::print(1, "✅");


	pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X Pos: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y Pos: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta/Heading: %f", chassis.getPose().theta);
            pros::lcd::print(3, "lb Pos: %f", ladyOdom.get_position()/100.0); 

            pros::delay(20);
        }
    });

    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });

    pros::Task antiJamTask([&]() {
        while (true) {
            if(inAuton == true){
                if(on){
                    if(direction){
                        if(chain.get_actual_velocity()<50 && chain.get_actual_velocity()>1){
                            chain.move(-127);
                            flexwheel.move(127);
                            pros::delay(100);
                            chain.move(127);
                        } else{
                            chain.move(127);
                            flexwheel.move(127);
                        }
                    } else{
                        chain.move(-127);
                        flexwheel.move(-127);
                    }
                } else{
                    chain.move(0);
                    flexwheel.move(0);
                }
            }
        }
    });
}

 

void stopTask(){}

void disabled(){}

void competition_initialize() {}

void intake(bool isOn, bool dir){
    on = isOn;
    direction = dir;
}


void autonomous() {
    inAuton = true;


    pros::delay(100);
	imu.set_heading(0);
    pros::delay(50);
	chassis.setPose(0,0,0);
    pros::delay(50);

    currState = 1;
    nextState();

    pros::delay(1000);

    nextState();    

    pros::delay(300);

    chassis.moveToPoint(0, -10, 500, {.forwards=false});
    chassis.turnToHeading(360 - 60, 500);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();

    chassis.moveToPoint(0, -22, 500, {.forwards=false, .maxSpeed=100});
    chassis.moveToPoint(0, -32, 300, {.forwards=false, .maxSpeed=50});

    pros::delay(500);
    mogo.extend();
    pros::delay(200);

    intake(true,true);

    // ring 2
    chassis.turnToHeading(360 - 84, 500);//CHANGE
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 20, 800, {.forwards=true});//CHANGE
    pros::delay(1000);

    // ring 3
    chassis.turnToHeading(360 - 87, 500);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 12.15, 500, {.forwards=true});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(0, 6, 500, {.forwards=false});

    // ring 4
    chassis.turnToHeading(30, 500); //change 
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 9.15, 800, {.forwards=true});
    chassis.waitUntilDone();
    pros::delay(1000); 
    chassis.moveToPose(0, 0, 0, 800, {.forwards=false});

    chassis.turnToHeading(360 - 118, 600);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    pros::delay(100); 
    
    chassis.moveToPose(0,40, 0, 700, {.forwards=true, .maxSpeed=80});
    chassis.moveToPose(0,70, 0, 400, {.forwards=true, .maxSpeed=30});

    intake(false, false);
} 

void opcontrol() {
    inAuton = false;
    currState = 0;

	while (true) {
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);

        int sensor_value = distance_sensor.get_distance();
		pros::delay(20);

        if(master.get_digital(DIGITAL_B)){
            mogo.extend();
        } else if(master.get_digital(DIGITAL_DOWN)){
            mogo.retract();
        }

        if(master.get_digital(DIGITAL_RIGHT)){
            if(sensor_value < 20.0){
                chain.move(-127);
                flexwheel.move(-127);
                pros::delay(100);
                chain.move(0);
                flexwheel.move(0);
            }
        }

        if(master.get_digital(DIGITAL_L1)){
            doinker.set_value(true);
        } else {
            doinker.set_value(false);
        }

        if(master.get_digital(DIGITAL_R1)){
            chain.move(127);
            flexwheel.move(127);
        }
        else if(master.get_digital(DIGITAL_R2)){
            chain.move(-127);
            flexwheel.move(-127);
        }
        else{
            chain.move(0);
            flexwheel.move(0);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            nextState();
        }

        // insert code for descore

	}
}