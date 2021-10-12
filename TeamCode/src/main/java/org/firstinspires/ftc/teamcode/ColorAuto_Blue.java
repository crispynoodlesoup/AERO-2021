package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* This is our autonomous, or LinearOpMode
 * This is where we make use of our other classes and tell the robot
 * exactly where it needs to go and what it needs to do
 */
@Autonomous(name="SkystoneBlue", group="generated")
public class ColorAuto_Blue extends LinearOpMode {
    // Objects and variables from our other custom classes
    ControlCalc calc = new ControlCalc();
    HardwareMecanumbot robot = new HardwareMecanumbot();\
    
    // Code that runs when the driver initializes
    @Override
    public void runOpMode(){
        robot.initDrive(this);
        calc.initDrive(robot);
        
        // Code that runs when the driver presses PLAY
        waitForStart();
        
        /* First, we'll be driving by time
         * This is the most basic auto you can make
         * It is not that accurate, but it usually gets the job done
         */
        
        // reset runtime, or measured time, to 0
        robot.runtime.reset();
        
        // check code should still be running, and drive forward until it's been more than one second
        // using the custom function moveMecanum written in HardwardMecanumbot file
        // set forward drive speed to 0.8
        robot.moveMecanum(0.8, 0, 0, 0);
        while(opModeIsActive() && !isStopRequested() && robot.runtime.seconds() < 1.0) {
            // telemetry writes data you give it to the driver station (phone) in real time
            // while waiting for this while loop to end, 
            telemetry.addData("Path", "Split 1: %2.5f S Elapsed", robot.runtime.seconds());
            telemetry.update();
        }
        robot.moveMecanum(0, 0, 0, 0); // stop robot

        // reset runtime, or measured time, to 0
        robot.runtime.reset();
        
        // check code should still be running, and turn until it's been more than one second
        robot.moveMecanum(0, 0, 0.5, 0);
        while(opModeIsActive() && !isStopRequested() && robot.runtime.seconds() < 1.0) {
            telemetry.addData("Path", "Split 2: %2.5f S Elapsed", robot.runtime.seconds());
            telemetry.update();
        }
        robot.moveMecanum(0, 0, 0, 0); // stop robot

        // reset runtime, or measured time, to 0
        robot.runtime.reset();
        
        // check code should still be running, and strafe until it's been more than one second
        robot.moveMecanum(0, 0.8, 0, 0);
        while(opModeIsActive() && !isStopRequested() && robot.runtime.seconds() < 1.0) {
            telemetry.addData("Path", "Split 3: %2.5f S Elapsed", robot.runtime.seconds());
            telemetry.update();
        }
        robot.moveMecanum(0,0,0,0); // make sure robot is stopped
        
        telemetry.addData("Drive by time path", "complete");
        telemetry.update();
        sleep(3000);
    }
}
