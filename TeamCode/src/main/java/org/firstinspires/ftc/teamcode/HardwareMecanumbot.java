package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/* This is a helper class that basically initializes the hardware and
 * gives us functions to control the power of each motor and servo
 */
public class HardwareMecanumbot {//access instruments of Hub
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angle;
    ColorSensor colorLeft;
    ColorSensor colorRight;

    // motor declarations
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor carousel, leftIntake, rightIntake, slides;
    public Servo dropPlate, cap, leftClaw, rightClaw, clamp, capX, capY;
    public CRServo capExtrude;
    
    //variables
    double frontL, frontR, backL, backR;
    public double encoderFL, encoderFR, encoderBL, encoderBR;
    public double ticksPerRotation;
    ElapsedTime rampTime = new ElapsedTime();
    ElapsedTime rightIntakeTime = new ElapsedTime();
    ElapsedTime leftIntakeTime = new ElapsedTime();

    public HardwareMecanumbot() {}
    
    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        //parameters we're sending to the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        frontLeft   = opMode.hardwareMap.get(DcMotor.class, "front_left");
        frontRight  = opMode.hardwareMap.get(DcMotor.class, "front_right");
        backLeft    = opMode.hardwareMap.get(DcMotor.class, "back_left");
        backRight   = opMode.hardwareMap.get(DcMotor.class, "back_right");
        carousel    = opMode.hardwareMap.get(DcMotor.class, "carousel");
        leftIntake  = opMode.hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = opMode.hardwareMap.get(DcMotor.class, "right_intake");
        slides      = opMode.hardwareMap.get(DcMotor.class, "slides");
        cap         = opMode.hardwareMap.get(Servo.class,   "cap");
        dropPlate   = opMode.hardwareMap.get(Servo.class,   "drop_plate");
        leftClaw    = opMode.hardwareMap.get(Servo.class,   "left_claw");
        rightClaw   = opMode.hardwareMap.get(Servo.class,   "right_claw");
        clamp       = opMode.hardwareMap.get(Servo.class,   "clamp");
        capExtrude  = opMode.hardwareMap.get(CRServo.class,   "capExtrude");
        capX        = opMode.hardwareMap.get(Servo.class,   "capX");
        capY        = opMode.hardwareMap.get(Servo.class,   "capY");

        colorLeft   = opMode.hardwareMap.get(ColorSensor.class, "color_left");
        colorLeft.enableLed(true);

        colorRight  = opMode.hardwareMap.get(ColorSensor.class, "color_right");
        colorRight.enableLed(true);
        
        // initialize the IMU for gyroscope and accelerometer
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //encoders for driving
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //brake the motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get number of encoder ticks
        ticksPerRotation = slides.getMotorType().getTicksPerRev();
    }

    // This function handles primary driving operation in TeleOP
    public void moveMecanum(double f, double s, double t, double vs) {
        // Add the vectors of motion for each wheel 'f' = forward, 't' = turn, 's' = strafe
        // Then scale for a value between [-1, 1] using powerRatio
        double powerRatio = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(t), 1);
        frontL = (f + t + s) / powerRatio;
        frontR = (f - t - s) / powerRatio;
        backL  = (f + t - s) / powerRatio;
        backR  = (f - t + s) / powerRatio;

        // Shorten the range of the power output to provide a more useful range of movement
        frontL *= Math.abs(frontL)*0.9;
        frontR *= Math.abs(frontR)*0.9;
        backL  *= Math.abs(backL)*0.9;
        backR  *= Math.abs(backR)*0.9;

        // Account for dead zone in the center of the joystick
        if(frontL < -0.0015) frontL -= 0.1; else if(frontL > 0.0015) frontL += 0.1; else frontL = 0;
        if(frontR < -0.0015) frontR -= 0.1; else if(frontR > 0.0015) frontR += 0.1; else frontR = 0;
        if(backL < -0.0015) backL -= 0.1; else if(backL > 0.0015) backL += 0.1; else backL = 0;
        if(backR < -0.0015) backR -= 0.1; else if(backR > 0.0015) backR += 0.1; else backR = 0;

        // Logic for Sneaking 'vs' = variable sneak
        // Essentially a brake for fine movement
        vs = Range.clip(vs, 0, 0.9 - 0.15*Math.abs(s/powerRatio) - 0.08*Math.abs(t/powerRatio));
        frontL = Range.clip(frontL, -1 + vs, 1 - vs);
        frontR = Range.clip(frontR, -1 + vs, 1 - vs);
        backL  = Range.clip(backL, -1 + vs, 1 - vs);
        backR  = Range.clip(backR, -1 + vs, 1 - vs);

        // Set power for drive motors
        frontLeft.setPower(frontL);
        frontRight.setPower(frontR);
        backLeft.setPower(backL);
        backRight.setPower(backR);
    }

    //returns a power value based on time from start
    public double powerRamp(double period, double startPow, double endPow) {
        if(rampTime.seconds() > period)
            return endPow;
        if(startPow > endPow)
            return Range.clip((rampTime.seconds() / period) * endPow, endPow, startPow);
        return Range.clip((rampTime.seconds() / period) * endPow, startPow, endPow);
    }

    // called to start automatic intake
    public void dropRightIntake() {
        if(rightIntakeTime.seconds() < 0.2) {
            rightIntake.setPower(-0.75);
        } else if(rightIntakeTime.seconds() < 0.5) {
            rightIntake.setPower(0);
            rightClaw.setPosition(0.368);
        } else
            rightIntake.setPower(-0.75);
    }

    // called to start automatic throughtake
    public void retractRightIntake(boolean isBlock) {
        if(rightIntakeTime.seconds() < 0.5) {
            rightIntake.setPower(0);
            rightClaw.setPosition(0.788);
        } else if(rightIntakeTime.seconds() < 1.6) {
            if(isBlock)
                rightIntake.setPower(0.44);
            else
                rightIntake.setPower(0.34);
        } else
            rightIntake.setPower(0);
    }

    // called to start automatic intake
    public void dropLeftIntake() {
        if(leftIntakeTime.seconds() < 0.2) {
            leftIntake.setPower(0.75);
        } else if(leftIntakeTime.seconds() < 0.6) {
            leftIntake.setPower(0);
            leftClaw.setPosition(0.362);
        } else
            leftIntake.setPower(0.75);
    }

    // called to start automatic throughtake
    public void retractLeftIntake(boolean isBlock) {
        if(leftIntakeTime.seconds() < 0.5) {
            leftIntake.setPower(0);
            leftClaw.setPosition(0);
        } else if(leftIntakeTime.seconds() < 1.6) {
            if(isBlock)
                leftIntake.setPower(-0.44);
            else
                leftIntake.setPower(-0.34);
        } else
            leftIntake.setPower(0);
    }

    public void setSlidePower(double power) {
        if(power > 0.05)
            slides.setPower(power);
        else if(getSlidePosition() < 2.5 && power < -0.05)
            slides.setPower(power);
        else
            slides.setPower(0);
    }

    public double getSlidePosition() {
        return slides.getCurrentPosition() / 752.0;
    }

    public void resetSlidePosition() {
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void homeSlides() {
        if(getSlidePosition() < -0.1)
            slides.setPower(-0.3);
        else if(getSlidePosition() > 1)
            slides.setPower(1);
        else if(getSlidePosition() > 0.25)
            slides.setPower(0.65);
        else if(getSlidePosition() > 0.02)
            slides.setPower(0.45);
        else slides.setPower(0);
    }

    //reading angle objects z axis
    public double getAngle() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }
    
    public void resetAngle() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
