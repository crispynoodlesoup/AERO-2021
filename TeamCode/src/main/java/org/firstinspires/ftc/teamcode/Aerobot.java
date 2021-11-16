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
        // Onitialize other classes with this OPmode
        robot.initDrive(this);
        calc.initDrive(robot);

        // Extra variables for movement
        double forward, turn, strafe, varSneak;
        boolean LBPrev = false, capToggle = false;

        // Init positions
        robot.cap.setPosition(0.87);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup variables to read the joystick values for this loop
            forward  = -gamepad1.left_stick_y;
            turn     =  gamepad1.right_stick_x;
            strafe   =  gamepad1.left_stick_x;
            varSneak =  gamepad1.right_trigger;
            
            // Move around
            robot.moveMecanum(forward, strafe, turn, varSneak);

            // logic for carousel
            if(gamepad1.x)
                robot.carousel.setPower(robot.powerRamp(0.75, 0, 1));
            else if(gamepad1.b)
                robot.carousel.setPower(robot.powerRamp(0.75, 0, -1));
            else {
                robot.rampTime.reset();
                robot.carousel.setPower(0);
            }

            // dropper
            if(gamepad1.right_bumper)
                robot.preload.setPosition(0);
            else
                robot.preload.setPosition(0.25);

            // capper
            if(gamepad1.left_bumper && !LBPrev)
                capToggle = !capToggle;
            if(capToggle)
                robot.cap.setPosition(0.68 - 0.36*gamepad1.left_trigger);
            else
                robot.cap.setPosition(0.865);
            LBPrev = gamepad1.left_bumper;

            // intake
            if(gamepad1.dpad_right)
                robot.rightIntake.setPower(-1);
            else
                robot.rightIntake.setPower(0);
            if(gamepad1.dpad_left)
                robot.leftIntake.setPower(-1);
            else
                robot.leftIntake.setPower(0);
        }
        robot.moveMecanum(0,0,0,0);
    }
}
