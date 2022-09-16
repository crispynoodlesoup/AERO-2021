package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/* This is the driver period class
 * First we read the joystick values
 * Then send those values to the Hardware Mecanumbot class
 * for the robot to turn into movement
 */
@TeleOp(name="AerobotManual", group="Linear Opmode")
public class AerobotManual extends LinearOpMode {
    // Declare runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Create Objects from our custom classes
    ControlCalc calc = new ControlCalc();
    HardwareMecanumbot robot = new HardwareMecanumbot();

    @Override
    public void runOpMode() {
        // Initialize objects with this OpMode
        robot.initDrive(this);
        calc.initDrive(robot);

        // Extra variables for movement
        double forward, turn, strafe, varSneak;
        double capPos = 0.2; //initialize position
        boolean P2RSBPrev, P2RSB = false; // P2RSB - Player 2 Right Stick Button
        boolean padTwoBPrev = false, padTwoB = false, rightClawToggle = false;
        boolean padTwoRightDPrev = false, padTwoRightD = false, leftClawToggle = false;

        // Init positions
        //robot.cap.setPosition(capPos);
        //robot.rightClaw.setPosition(0.58);
        //robot.leftClaw.setPosition(0.5);
        robot.resetSlidePosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // -------------------------  MOVEMENT  -------------------------
            // Setup variables to read the joystick values for this loop
            forward = -gamepad1.left_stick_y;
            turn = -gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
            varSneak = gamepad1.right_trigger;

            // Use built in hardware function to determine drive power
            robot.moveMecanum(forward, strafe, turn, varSneak);

            // ------------------------  OLD INTAKE  ------------------------
            padTwoBPrev =  padTwoB;
            padTwoB     =  gamepad2.b;
            padTwoRightDPrev =  padTwoRightD;
            padTwoRightD     =  gamepad2.dpad_right;

            // Left side claw
            if(padTwoRightD && !padTwoRightDPrev)
                leftClawToggle = !leftClawToggle;
            if(leftClawToggle)
                robot.leftClaw.setPosition(0.42);
            else
                robot.leftClaw.setPosition(0);

            // Left side intake
            if(gamepad2.dpad_down && robot.colorLeft.green() < 500) { // left side intake
                robot.leftIntake.setPower(0.6);
                robot.colorLeft.enableLed(false);
            } else if(gamepad2.dpad_up) // reverse to drop freight
                robot.leftIntake.setPower(-0.6);
            else { // stop when no button is pressed
                robot.leftIntake.setPower(0);
                robot.colorLeft.enableLed(true);
            }

            // Right side claw
            if(padTwoB && !padTwoBPrev)
                rightClawToggle = !rightClawToggle;
            if(rightClawToggle)
                robot.rightClaw.setPosition(0.368);
            else
                robot.rightClaw.setPosition(0.795);

            // Right side intake
            if(gamepad2.a && robot.colorRight.green() < 500) { // right side intake
                robot.rightIntake.setPower(-0.6);
                robot.colorRight.enableLed(false);
            } else if(gamepad2.y) // reverse to drop freight
                robot.rightIntake.setPower(0.7);
            else { // stop when no button is pressed
                robot.rightIntake.setPower(0);
                robot.colorRight.enableLed(true);
            }

            // --------------------------  SLIDES  --------------------------
            // left stick y makes it slidey
            robot.setSlidePower(-gamepad2.left_stick_y);

            // button to home slides
            if (gamepad2.left_stick_button)
                robot.homeSlides();

            // reset home position
            P2RSBPrev = P2RSB;
            P2RSB = gamepad2.right_stick_button;
            if (P2RSB && !P2RSBPrev)
                robot.resetSlidePosition();

            // --------------------------  DEPOSIT  --------------------------
            // Drop plate extension
            if (gamepad2.x)
                robot.dropPlate.setPosition(0.7);
            else
                robot.dropPlate.setPosition(0.05);

            // -------------------------  CAROUSEL  -------------------------
            if (gamepad2.b)
                robot.carousel.setPower(robot.powerRamp(0.6, 0.1, 0.75));
            else if (gamepad2.dpad_right)
                robot.carousel.setPower(robot.powerRamp(0.6, -0.1, -0.75));
            else {
                robot.rampTime.reset();
                robot.carousel.setPower(0);
            }

            // -------------------------  CAPPING  -------------------------
            if (gamepad1.dpad_right)
                capPos = 0.3;
            if (gamepad1.dpad_up)
                capPos = Range.clip(capPos + 0.004, 0.2, 0.865);
            else if (gamepad1.dpad_down)
                capPos = Range.clip(capPos - 0.004, 0.2, 0.865);
            robot.cap.setPosition(capPos);

            // ------------------------  TELEMETRY  ------------------------
            // Right Side Color Sensor
            if (robot.colorRight.green() > 500) {
                if (robot.colorRight.blue() * 1.5 < robot.colorRight.green())
                    telemetry.addData("Right Side Freight", "Block");
                else
                    telemetry.addData("Right Side Freight", "Ball");
            } else
                telemetry.addData("Right Side Freight", "None");

            // Left Side Color Sensor
            if (robot.colorLeft.green() > 500) {
                if (robot.colorLeft.blue() * 1.5 < robot.colorLeft.green())
                    telemetry.addData("Left Side Freight", "Block");
                else
                    telemetry.addData("Left Side Freight", "Ball");
            } else
                telemetry.addData("Left Side Freight", "None");

            telemetry.addLine();

            telemetry.addData("slide encoder ticks", robot.ticksPerRotation);
            telemetry.addData("current slide encoder tick", robot.slides.getCurrentPosition());
            telemetry.addData("slide position", robot.getSlidePosition()); // slide position\

            telemetry.update(); // update all data
        }

        robot.moveMecanum(0, 0, 0, 0);
    }
}