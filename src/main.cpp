#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"



/*
1. Reattach odom sensors, upload code, run driver control, see if any sensors are flipped and all are connected to proper/working ports and if they are all working properly
2. If they aren't try running the same code for blue right or your prior repos, which haven't been changed
3. Measure offset of tracking wheel
4. Test auton
5. Tune pid
6. Test lady brown odom, see if its printing proper brian values
7. test lady brown
8. check wheel sizes, rpm, etc.
9. check if brain is working
 */



int positionState = 0;
const int numStates = 3;
int states[numStates] = {0, 27, 150};
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

//! Make sure you test both odom sensors, imu sensor after adding the sensors
pros::Imu imu(16); 
pros::Distance distance_sensor(9);

//! Make sure you test both odom sensors, imu sensor after adding the sensors(test both ports to see if they are reversed)
pros::Rotation yOdom(15); 
pros::Rotation ladyOdom(21); 


lemlib::TrackingWheel vertical_tracking_wheel(&yOdom, lemlib::Omniwheel::NEW_2, -2.5); //! Measure offset

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, 
    nullptr,
    nullptr,
    nullptr,
    &imu
);


// PID(After you tune, everything should work as expect, if not look at the printscreens to see if the robot thinks its getting to such positions, if so, then its likely a PID tuning issue, will take sometime but you have time)
// Do default, set 0, slowly adjust each factor, 
// at the end, should have smooth motions


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

// create the chassis
lemlib::Chassis chassis(
    drivetrain, 
    lateral_controller,
    angular_controller, 
    sensors, 
    &driveCurve,
    &driveCurve
);

void nextState(){
    currState += 1;
    if(currState == 3){
        currState = 0;
    }
    target = states[currState];
}

void liftControl(){
    double kp = 3;
    double err = target - (ladyOdom.get_position()/100.0);
    double velocity = kp * err;
    lb.move(velocity);
}


void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	imu.set_heading(0);
    pros::delay(100);

    yOdom.reset_position(); 
    ladyOdom.reset_position();
	pros::delay(100);

    chassis.calibrate();
	pros::delay(500);

	pros::lcd::print(1, "Calibration Complete");

    //! Copy THis ->

	pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "lb: %f", ladyOdom.get_position()/100.0); // heading
            pros::delay(20);
        }
    });
    //!_________________________


    
    pros::Task liftControlTask([]{
        while(true){
            liftControl();
            pros::delay(10);
        }
    });
}

void disabled() {} // NOT NEEDED!!!

void competition_initialize() {} // NOT NEEDED!

void autonomous() {
	imu.set_heading(0);
	chassis.setPose(0,0,180);

    //mogo
    chassis.moveToPoint(0, 33, 2000, {.forwards=false, .maxSpeed=85});
    pros::delay(800);
    mogo.extend();
    chain.move(200);
    pros::delay(400);
    flexwheel.move(200);

    //ring 0
    chassis.turnToHeading(360 - 280,700);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 18, 3000, {.forwards=true, .maxSpeed=85});
    chassis.waitUntilDone();

    //ring 1
    chassis.turnToHeading(360 - 80, 600);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 11, 1000, {.forwards=true, .maxSpeed=85});
    chassis.waitUntilDone();
    pros::delay(200); 
    chassis.moveToPoint(0, -5, 600, {.forwards=false, .maxSpeed=85});

    //ring 2
    chassis.turnToHeading(360 - 340, 500);    
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 18, 1300, {.forwards=true, .maxSpeed=85});
    pros::delay(1500);
    chassis.moveToPoint(0, -16.5, 1300, {.forwards=false, .maxSpeed=85});
    chassis.waitUntilDone();
    pros::delay(500);
    mogo.retract();
    chassis.turnToHeading(360 - 112,5000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 23.75, 1000, {.forwards=true, .maxSpeed=65});
    chassis.waitUntilDone();
    doinker.extend();
    pros::delay(100);
    chassis.moveToPoint(0, -0.5, 1000, {.forwards=false, .maxSpeed=65});
    chassis.waitUntilDone();
    doinker.retract();
    chassis.waitUntilDone();
    chassis.turnToHeading(360 - 20, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 20, 1000, {.forwards=true, .maxSpeed=85});
    pros::delay(500);






    // pros::delay(900);
    // chain.move(0);


    // //ring 3
    // chassis.turnToHeading(250, 500);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, 28, 3000, {.forwards=true, .maxSpeed=65});
} 

void opcontrol() {
	while (true) {

        // movement
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);

        int sensor_value = distance_sensor.get_distance();
		pros::delay(20); // Run for 20 ms then update

        //! Mogo Clamp
        if(master.get_digital(DIGITAL_B)){
            mogo.extend();
        } else if(master.get_digital(DIGITAL_DOWN)){
            mogo.retract();
        }


        //! Digital Sensor
        if(master.get_digital(DIGITAL_RIGHT)){
            if(sensor_value < 10.0){
                chain.move(-400);
                flexwheel.move(-400);
                pros::delay(800);  // Delay for 500ms (duration of the outtake)
                chain.move(0);
                flexwheel.move(0);
            }
        }

        //! Doinker
        if(master.get_digital(DIGITAL_L1)){
            doinker.set_value(true);
        } else {
            doinker.set_value(false);
        }


        //! Intake
        if(master.get_digital(DIGITAL_R1)){
            chain.move(400);
            flexwheel.move(400);
        } else if(master.get_digital(DIGITAL_R2)){
            chain.move(-400);
            flexwheel.move(-400);
        } else{
            chain.move(0);
            flexwheel.move(0);
        }

        //! Lady Brown
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            nextState();
        }
	}
}