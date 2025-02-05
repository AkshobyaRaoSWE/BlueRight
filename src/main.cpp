#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"



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
10. CHECK SENSORS!!!
11. TRY NORMAL VERSION WITH JUST COLOR SORT, INTAKE AND STUFF IS IN NORMAL OP CONTROL, TASK WITH THE VOID THINGIE
12. We would do current version with everything in task or only color sort(only if or if/else + if)
 */



int positionState = 0;
const int numStates = 3;
int states[numStates] = {0, 28, 150};
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

pros::Imu imu(16); 
pros::Distance distance_sensor(9);
pros::Rotation yOdom(15); 
pros::Rotation ladyOdom(21); 
pros::Optical colorSensor(7);
lemlib::TrackingWheel vertical_tracking_wheel(&yOdom, lemlib::Omniwheel::NEW_2, -2.5); 

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, 
    nullptr,
    nullptr,
    nullptr,
    &imu
);


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

// void colorSort(){
//     while(true){
//         if(colorSensor.get_hue()>5 && colorSensor.get_hue()<15){
//             chain.move(-400);
//             pros::delay(100);  // Delay for 500ms (duration of the outtake)
//             chain.move(0);
//         }
//     }
// }

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	imu.set_heading(0);
    pros::delay(100);

    yOdom.reset_position(); 
    pros::delay(100);

    ladyOdom.reset_position();
	pros::delay(100);

    chassis.calibrate();
	pros::delay(500);
    colorSensor.set_integration_time(300);
    colorSensor.get_led_pwm();
	pros::lcd::print(1, "Calibration Complete");


	pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "lb: %f", ladyOdom.get_position()/100.0); // heading
            // pros::lcd::print(4, "color: %f", colorSensor.get_hue()/100.0); // heading

            pros::delay(20);
        }
    });
    
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
    chassis.moveToPoint(0, 22, 1000, {.forwards=false, .maxSpeed=85});
    chassis.moveToPoint(0, 36, 1000, {.forwards=false, .maxSpeed=50});

    pros::delay(1000);
    mogo.extend();

    // ring 1
    pros::delay(100);
    
    
    chain.move(200);
    flexwheel.move(200);


    //ring 2
    chassis.turnToHeading(360 - 270, 600);//CHANGE
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 18, 1000, {.forwards=true, .maxSpeed=100});//CHANGE
    pros::delay(100);
    //ring 3
    chassis.turnToHeading(360 - 85, 600);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 12, 1000, {.forwards=true, .maxSpeed=350});
    chassis.waitUntilDone();
    pros::delay(200); 
    chassis.moveToPoint(0, 6, 600, {.forwards=false, .maxSpeed=350});

    //ring 4
    chassis.turnToHeading(360 - 325, 500); //change 
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 1300, {.forwards=true, .maxSpeed=350});
    chassis.waitUntilDone();
    chassis.moveToPose(0, -20, 0, 1000, {.forwards=false, .maxSpeed=350});
    chassis.waitUntilDone();
    chassis.turnToHeading(-100, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});

    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();

    chassis.moveToPose(0, 50, 0, 1000, {.forwards=true, .maxSpeed=40});
    chassis.waitUntilDone();
    chain.move(0);
    flexwheel.move(0);
    nextState();
    pros::delay(100);
    nextState();
    // test comment
    // pros::delay(200);
    // doinker.extend();
    // pros::delay(200);
    // chassis.moveToPose(0, 16, 0, 1000, {.forwards=true, .minSpeed=100});
    // pros::delay(300);
    // chassis.turnToHeading(-100, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed=60});
    // pros::delay(200);
    // doinker.retract();
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPose(0, -5, 0, 1000, {.forwards=true, .maxSpeed=80});
    // chassis.turnToHeading(20, 1000);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPose(0, 12, 0, 1000, {.forwards=true, .maxSpeed=80});

    // pros::delay(1000);
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    // chassis.moveToPose(0, 13, 0, 1000, {.forwards=false, .minSpeed=100});
    // chassis.moveToPose(0, 22, 0, 1000, {.forwards=true, .minSpeed=100});
    
    // pros::delay(100);
    // mogo.retract();
    // chain.move(0);


    // chassis.turnToHeading(-147,800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, 15, 900, {.forwards=true, .maxSpeed=85});
    // pros::delay(900);
    // doinker.extend();
    // pros::delay(200);
    // chassis.moveToPoint(0, 7, 1000, {.forwards=false, .maxSpeed=70});
    // pros::delay(200);
    // doinker.retract();
    // pros::delay(100);
    // chassis.turnToHeading(19,600);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    
    // chassis.moveToPoint(0, 20, 1000, {.forwards=true, .maxSpeed=150});
    // chassis.waitUntilDone();
    // pros::delay(100);
    // doinker.extend();
    // pros::delay(100);
    // chassis.turnToHeading(112, 2000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});


    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, -25, 1000, {.forwards=false, .maxSpeed=150});
    // pros::delay(200);
    // chain.move(400);
    // chassis.waitUntilDone();
    // pros::delay(200);
    // chain.move(0);
    // chassis.moveToPoint(0,45,1000,{.forwards=true, .maxSpeed=70});


} 

void opcontrol() {
	while (true) {

		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);

        int sensor_value = distance_sensor.get_distance();
		pros::delay(20); // Run for 20 ms then update

        if(master.get_digital(DIGITAL_B)){
            mogo.extend();
        } else if(master.get_digital(DIGITAL_DOWN)){
            mogo.retract();
        }


        if(master.get_digital(DIGITAL_RIGHT)){
            if(sensor_value < 10.0){
                chain.move(-400);
                flexwheel.move(-400);
                pros::delay(100);  // Delay for 500ms (duration of the outtake)
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
            chain.move(400);
            flexwheel.move(400);
        }
        else if(master.get_digital(DIGITAL_R2)){
            chain.move(-400);
            flexwheel.move(-400);
        }
        else if(master.get_digital(DIGITAL_Y)){
            chain.move(-50);
        } else{
            chain.move(0);
            flexwheel.move(0);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            nextState();
        }
	}
}