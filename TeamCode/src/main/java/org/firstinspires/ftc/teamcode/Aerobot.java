package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/* This is the driver period class
 * First we read the joystick values
 * Then send those values to the Hardware Mecanumbot class
 * for the robot to turn into movement
 */
@TeleOp(name="Aerobot", group="Linear Opmode")
public class Aerobot extends LinearOpMode {
    // Declare runtime
    private ElapsedTime runtime = new ElapsedTime();
    
    // Create Objects from our custom classes
    ControlCalc calc = new ControlCalc();
    HardwareMecanumbot robot = new HardwareMecanumbot();
    
    @Override
    public void runOpMode() {
        // initialize other classes with this OPmode
        robot.initDrive(this);
        calc.initDrive(robot);

        // variables for joystick
        double forward, turn, strafe, varSneak;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup variables to read the joystick values for this loop
            forward   = -gamepad1.left_stick_y;
            turn      =  gamepad1.right_stick_x;
            strafe    =  gamepad1.left_stick_x;
            varSneak  =  gamepad1.right_trigger;
            
            // Move around
            robot.moveMecanum(forward, strafe, turn, varSneak);
        }
        robot.moveMecanum(0,0,0,0);
    }
}
