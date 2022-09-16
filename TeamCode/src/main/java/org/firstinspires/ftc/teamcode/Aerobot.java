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
@TeleOp(name="Aerobot", group="Linear Opmode")
public class Aerobot extends LinearOpMode {
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
        double  forward, turn, strafe, varSneak;
        double  capPos = 0.01;
        double  capX = 0.2, capY = 0;
        boolean P1BackPrev, P1Back = false, capExtrudeToggle = true;
        boolean P2RBPrev, P2RB = false, rightIntakeToggle = false; // P2RB - Player 2 Right Bumper
        boolean P2LBPrev, P2LB = false, leftIntakeToggle = false; // P2RB - Player 2 Left Bumper
        boolean P2RSBPrev, P2RSB = false; // P2RSB - Player 2 Right Stick Button
        boolean P2XPrev, P2X = false; // P2RSB - Player 2 X
        boolean P2APrev, P2A = false;
        boolean P2BPrev, P2B = false;
        boolean P2YPrev, P2Y = false;
        boolean clampToggle = false;
        double  clampPos = 0.1;
        boolean rightBlock = true, leftBlock = true; // stores the last type  of freight collected
        double  rightGreen = 0, rightGreenPrev;
        double  leftGreen = 0, leftGreenPrev;

        //old intake variables
        //boolean padTwoBPrev = false, padTwoB = false, rightClawToggle = false;
        //boolean padTwoRightDPrev = false, padTwoRightD = false, leftClawToggle = false;

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
            forward = gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_y;
            varSneak = gamepad1.right_trigger;

            // Use built in hardware function to determine drive power
            robot.moveMecanum(forward, strafe, turn, varSneak);

            // --------------------------  INTAKE  --------------------------
            // *** right side *** //
            P2RBPrev = P2RB;
            P2RB = gamepad2.right_bumper;
            rightGreenPrev = rightGreen;
            rightGreen = robot.colorRight.green();

            if (P2RB && !P2RBPrev) {
                rightIntakeToggle = !rightIntakeToggle;
                robot.rightIntakeTime.reset();
            }

            // if ball collected, automatically start intaking
            if (rightGreen > 500 && rightGreenPrev < 500 && gamepad2.right_trigger < 0.6) {
                rightIntakeToggle = false;
                robot.rightIntakeTime.reset();
                rightBlock = robot.colorRight.blue() * 1.5 < robot.colorRight.green();
            }

            if (rightIntakeToggle)
                robot.dropRightIntake();
            else {
                robot.retractRightIntake(rightBlock);
            }

            // *** left side *** //
            P2LBPrev = P2LB;
            P2LB = gamepad2.left_bumper;
            leftGreenPrev = leftGreen;
            leftGreen = robot.colorLeft.green();

            if (P2LB && !P2LBPrev) {
                leftIntakeToggle = !leftIntakeToggle;
                robot.leftIntakeTime.reset();
            }

            // if ball collected, automatically start intaking
            if (leftGreen > 500 && leftGreenPrev < 500 && gamepad2.right_trigger < 0.6) {
                leftIntakeToggle = false;
                robot.leftIntakeTime.reset();
                leftBlock = robot.colorLeft.blue() * 1.5 < robot.colorLeft.green();
            }

            if (leftIntakeToggle)
                robot.dropLeftIntake();
            else {
                robot.retractLeftIntake(leftBlock);
            }

            // --------------------------  SLIDES  --------------------------
            // right stick y makes it slidey
            robot.setSlidePower(gamepad2.left_stick_y);

            // homing button for the slide encoder
            P2RSBPrev = P2RSB;
            P2RSB = gamepad2.right_stick_button;
            if (P2RSB && !P2RSBPrev)
                robot.resetSlidePosition();

            // homing
            if (gamepad2.left_stick_button)
                robot.homeSlides();

            // --------------------------  DEPOSIT  --------------------------

            // Drop plate extension
            P2XPrev = P2X;
            P2X = gamepad2.x;
            if (P2X && !P2XPrev)
                clampToggle = false;
            else if(!P2X && P2XPrev) {
                robot.dropPlate.setPosition(0.03);
                clampPos = 0.1;
            }

            //different positions
            P2APrev = P2A;
            P2A = gamepad2.a;
            if(P2A && !P2APrev) {
                robot.dropPlate.setPosition(0.82);
                clampToggle = true;
                clampPos = 0.36;
            }
            P2BPrev = P2B;
            P2B = gamepad2.b;
            if(P2B && !P2BPrev) {
                robot.dropPlate.setPosition(0.7);
                clampToggle = true;
                clampPos = 0.325;
            }
            P2YPrev = P2Y;
            P2Y = gamepad2.y;
            if(P2Y && !P2YPrev) {
                robot.dropPlate.setPosition(0.6);
                clampToggle = true;
                clampPos = 0.24;
            }

            if(clampToggle)
                robot.clamp.setPosition(0.5);
            else
                robot.clamp.setPosition(clampPos);

            // -------------------------  CAROUSEL  -------------------------
            if (gamepad2.dpad_left) // blue side
                robot.carousel.setPower(robot.powerRamp(0.6, 0.1, 0.75));
            else if (gamepad2.dpad_right) // red side
                robot.carousel.setPower(robot.powerRamp(0.6, -0.1, -0.75));
            else {
                robot.rampTime.reset();
                robot.carousel.setPower(0);
            }

            // -------------------------  CAPPING  -------------------------
            if(gamepad1.dpad_left) // place cap
                capPos = 0.5;
            if(gamepad1.dpad_right) // pick up cap
                capPos = 0.72;
            if (gamepad1.dpad_up)
                capPos = Range.clip(capPos - 0.01, 0, 0.82);
            else if (gamepad1.dpad_down)
                capPos = Range.clip(capPos + 0.01, 0, 0.82);
            robot.cap.setPosition(capPos);

            // ----------------------  DOUBLE CAPPING  ----------------------
            // cap X movement
            if(gamepad1.x)
                capX = Range.clip(capX - 0.005, 0, 1);
            else if(gamepad1.b)
                capX = Range.clip(capX + 0.005, 0, 1);
            robot.capX.setPosition(capX);

            // cap Y movement
            if(gamepad1.y)
                capY = Range.clip(capY + 0.005, 0, 1);
            else if(gamepad1.a)
                capY = Range.clip(capY - 0.005, 0, 1);
            robot.capY.setPosition(capY);


            if(gamepad1.back)
                capExtrudeToggle = true;
            if(gamepad1.start)
                capExtrudeToggle = false;

            if(gamepad1.left_trigger > 0.05 && capExtrudeToggle)
                robot.capExtrude.setPower(gamepad1.left_trigger);
            else if(gamepad1.left_trigger > 0.05 && !capExtrudeToggle)
                robot.capExtrude.setPower(-gamepad1.left_trigger);
            else
                robot.capExtrude.setPower(0);

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

            /* ------------------------  OLD INTAKE  ------------------------
            padTwoBPrev =  padTwoB;
            padTwoB     =  gamepad2.b;
            padTwoRightDPrev =  padTwoRightD;
            padTwoRightD     =  gamepad2.dpad_right;

            // left side claw
            if(padTwoRightD && !padTwoRightDPrev)
                leftClawToggle = !leftClawToggle;
            if(leftClawToggle)
                robot.leftClaw.setPosition(0.94);
            else
                robot.leftClaw.setPosition(0.5);

            if(gamepad2.dpad_down && robot.colorLeft.green() < 1000) { // left side intake
                robot.leftIntake.setPower(-1);
                robot.colorLeft.enableLed(false);
            }
            else if(gamepad2.dpad_up) // reverse to drop freight
                robot.leftIntake.setPower(0.65);
            else { // stop when no button is pressed
                robot.leftIntake.setPower(0);
                robot.colorLeft.enableLed(true);
            }

            // new intake - right side claw
            if(padTwoB && !padTwoBPrev)
                rightClawToggle = !rightClawToggle;
            if(rightClawToggle)
                robot.rightClaw.setPosition(0.15);
            else
                robot.rightClaw.setPosition(0.58);

            if(gamepad2.a && robot.colorRight.green() < 1000) { // right side intake
                robot.rightIntake.setPower(1);
                robot.colorRight.enableLed(false);
            }
            else if(gamepad2.y) // reverse to drop freight
                robot.rightIntake.setPower(-0.65);
            else { // stop when no button is pressed
                robot.rightIntake.setPower(0);
                robot.colorRight.enableLed(true);
            }

            if(gamepad2.right_trigger > 0.1)
                robot.setSlidePower(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0.1)
                robot.setSlidePower(-gamepad2.left_trigger);
            else
                robot.setSlidePower(0);

            //drop plate
            if(gamepad2.x)
                robot.dropPlate.setPosition(0.6);
            else
                robot.dropPlate.setPosition(0.02);

            telemetry.addData("motor position", robot.getSlidePosition());
            telemetry.update();
            */
        }
        robot.moveMecanum(0, 0, 0, 0);
    }
}