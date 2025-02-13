#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"


int positionState = 0;
const int numStates = 3;

int statesAuton[numStates] = {-30, -2, 230};
int statesDriver[numStates] = {0, 34, 180};

int currState = 0;
int target = 0;

bool on = false;
bool direction = true;
bool auton = true;

bool isDescore = true;

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

    if(currState == 3 && auton == true){
        currState = 0;
    }

    if (auton == true) {
        target = statesAuton[currState];
    } else {
        target = statesDriver[currState];
    }
}


void liftControl(){
    double kp;
    if(auton == true){
        kp = 0.5;
    }
    else{
        kp = 2.5;
    }
    double err = target - (ladyOdom.get_position()/100.0);
    double velocity = kp * err;
    lb.move(velocity);
}

// void nextState2(){
//     currState2 += 1;
//     if(currState2 == 3){
//         currState2 = 0;
//     }
//     target2 = states2[currState2];
// }

// void liftControl2(){
//     double kp = 5;
//     double err = target2 - (ladyOdom.get_position()/100.0);
//     double velocity = kp * err;
//     lb.move(velocity);
// }

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	imu.set_heading(0);
    pros::delay(100);

    ladyOdom.reset_position();

    yOdom.reset_position(); 
    pros::delay(100);

    left_mg.set_brake_mode(pros::MotorBrake::brake);
    right_mg.set_brake_mode(pros::MotorBrake::brake);

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
            pros::lcd::print(4, "task:", currState); // heading

            pros::delay(20);

            liftControl();

            pros::delay(10);
            if(auton){
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

    // pros::Task liftControlTask2([]{
    //     while(true){
    //         liftControl2();
    //         pros::delay(10);
    //         if(on){
    //             if(direction){
    //                 if(chain.get_actual_velocity()<50 && chain.get_actual_velocity()>1){
    //                     chain.move(-127);
    //                     flexwheel.move(127);
    //                     pros::delay(100);
    //                     chain.move(127);
    //                 } else{
    //                     chain.move(127);
    //                     flexwheel.move(127);
    //                 }
    //             } else{
    //                 chain.move(-127);
    //                 flexwheel.move(-127);
    //             }
    //         } else{
    //             chain.move(0);
    //             flexwheel.move(0);
    //         }
    //     }
    // });

}

 

void stopTask(){

}
void disabled() {}

void competition_initialize() {}

void in(bool onn, bool dir){
    on = onn;
    direction=dir;
}


void autonomous() {
    
    // pros::Task liftControlTask2([]{
    //     while(true){
    //         liftControl2();
    //         pros::delay(10);
    //         if(on){
    //             if(direction){
    //                 if(chain.get_actual_velocity()<50 && chain.get_actual_velocity()>1){
    //                     chain.move(-127);
    //                     flexwheel.move(127);
    //                     pros::delay(100);
    //                     chain.move(127);
    //                 } else{
    //                     chain.move(127);
    //                     flexwheel.move(127);
    //                 }
    //             } else{
    //                 chain.move(-127);
    //                 flexwheel.move(-127);
    //             }
    //         } else{
    //             chain.move(0);
    //             flexwheel.move(0);
    //         }
    //     }
    // });
    auton = true;

    pros::delay(100);
	imu.set_heading(0);
	chassis.setPose(0,0,0);

    currState = 1;
    nextState();

    pros::delay(1000);


    currState = 2;

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


    // ring 1
    pros::delay(200);

    in(true,true);


    // // ring 2
    chassis.turnToHeading(360 - 84, 500);//CHANGE
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 20, 800, {.forwards=true});//CHANGE
    pros::delay(1000);

    // // ring 3
    chassis.turnToHeading(360 - 87, 500);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 12.15, 500, {.forwards=true});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(0, 6, 500, {.forwards=false});

    // // ring 4
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

    in(false, false);



    // chassis.waitUntilDone();
    // chassis.turnToHeading(-100, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});


    // // touch ladder
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPose(0, 50, 0, 1000, {.forwards=true, .maxSpeed=40});
    // chassis.waitUntilDone();
    // chain.move(0);
    // flexwheel.move(0);
    // nextState();
    // pros::delay(100);
    // nextState();
} 

void opcontrol() {
    // pros::Task liftControlTask([]{
    //     while(true){
    //         liftControl();
    //         pros::delay(10);
    //     }
    // });

    auton = false;
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
                chain.move(-400);
                flexwheel.move(-400);
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
            chain.move(400);
            flexwheel.move(400);
        }
        else if(master.get_digital(DIGITAL_R2)){
            chain.move(-400);
            flexwheel.move(-400);
        }
        else{
            chain.move(0);
            flexwheel.move(0);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            nextState();
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            if(currState == 0){
                nextState();
                nextState();
            } else if(currState == 1){
                nextState();
            }
        } else{
            currState = 2;
            nextState();
        }
	}
}