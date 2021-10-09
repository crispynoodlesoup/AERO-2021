package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* This is our autonomous, or LinearOpMode
 * This is where we make use of our other classes and tell the robot
 * exactly where it needs to go and what it needs to do
 */
@Autonomous(name="SkystoneBlue", group="generated")
public class ColorAuto_Blue extends LinearOpMode {
    // Objects and variables
    ControlCalc calc = new ControlCalc();
    HardwareMecanumbot robot = new HardwareMecanumbot();
    
    // Code that runs when the driver initializes
    @Override
    public void runOpMode(){
        robot.initDrive(this);
        calc.initDrive(robot);
        
        // Code that runs when the driver presses PLAY
        waitForStart();
        robot.runtime.reset();
        while( !isStopRequested() ) {
            calc.calcTurnPower(0);
            robot.updateEncoderValues();

            telemetry.addData("encoder ticks per rev", robot.encoderFL);
            telemetry.addData("milliseconds delta", calc.milliseconds - robot.runtime.milliseconds());
            telemetry.update();
        }
        robot.moveMecanum(0,0,0,0); // Stop the robot
    }
}
