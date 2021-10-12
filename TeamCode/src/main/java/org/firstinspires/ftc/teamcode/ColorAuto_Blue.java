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
        robot.moveMecanum(0, 0, -0.5, 0);
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
        
        //end of drive by time path
        telemetry.addData("Drive by time path", "complete");
        telemetry.update();
        sleep(3000);
        
        /* Next, we'll be driving using encoders
         * You'll need to look at the functions in hardwareMecanumbot to see the full picture
         * It should be fairly accurate, but we would need to make some adjustments
         * Before we can make a fairly consistent autonomous
         *
         * however, one of the biggest shortcomings of driving by encoders we still have:
         * If the wheels are not on the floor, the encoders can't detect that.
         * Imagine one wheel gets less traction than the other 3,
         * the encoder will just think the robot is moving straight,
         * when in reality we're TOKYO DRIFTING on all the other robots
         */
        
        driveInches(0.5, 5, 3);
        driveInches(-0.6, 20, 10);
        driveInches(0.2, 4, 5);
    }
    
    public void driveInches(double power, double inches, double timeout) {
        // check that the OpMode is still active
        if(opModeIsActive()) {
            // reset measurements
            robot.runtime.reset();
            robot.resetDriveEncoders();
            
            // set power until path complete or runtime exceeds timeout
            // also, when getting close to the target, slow down
            while(robot.runtime() < timeout && !isStopRequested() && Math.abs(robot.forwardAverage()) < inches) {
                robot.moveMecanum(power, 0, 0, 0);
                power = range.clip(power, Math.abs(robot.forwardAverage()) - inches, inches - Math.abs(robot.forwardAverage()));
                
                //log values for user
                telemetry.addData("Running to %2.5f", inches);
                telemetry.addData("at %2.5f", robot.forwardAverage();
            }
            robot.moveMecanum(0, 0, 0, 0);
            sleep(100);
        }
    }
}
