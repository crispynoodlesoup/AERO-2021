package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    
    // motor declarations
    public DcMotor frontLeft   = null;
    public DcMotor frontRight  = null;
    public DcMotor backLeft    = null;
    public DcMotor backRight   = null;
    public DcMotor carousel    = null;
    public DcMotor leftIntake  = null;
    public DcMotor rightIntake = null;

    public Servo preload = null;
    public Servo cap     = null;
    
    //variables
    double frontL, frontR, backL, backR;
    double ticksPerRotation;
    public double encoderFL, encoderFR, encoderBL, encoderBR;
    ElapsedTime rampTime = new ElapsedTime();

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
        preload     = opMode.hardwareMap.get(Servo.class,   "preload");
        cap         = opMode.hardwareMap.get(Servo.class,   "cap");
        
        // initialize the IMU for gyroscope and accelerometer
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Encoders for odometry
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //encoders for driving
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //brake the motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // get number of encoder ticks
        ticksPerRotation = frontLeft.getMotorType().getTicksPerRev();
    }
    
    public void moveMecanum(double f, double s, double t, double vs) {
        // math for mecanum wheels 'f' = forward, 't' = turn, 's' = strafe
        double powerRatio = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(t), 1);
        frontL = (f + t + s) / powerRatio;
        frontR = (f - t - s) / powerRatio;
        backL  = (f + t - s) / powerRatio;
        backR  = (f - t + s) / powerRatio;

        // some math to make joystick a better range of motion over the robot
        frontL *= Math.abs(frontL)*0.9;
        frontR *= Math.abs(frontR)*0.9;
        backL  *= Math.abs(backL)*0.9;
        backR  *= Math.abs(backR)*0.9;
        if(frontL < -0.0015) frontL -= 0.1; else if(frontL > 0.0015) frontL += 0.1; else frontL = 0;
        if(frontR < -0.0015) frontR -= 0.1; else if(frontR > 0.0015) frontR += 0.1; else frontR = 0;
        if(backL < -0.0015) backL -= 0.1; else if(backL > 0.0015) backL += 0.1; else backL = 0;
        if(backR < -0.0015) backR -= 0.1; else if(backR > 0.0015) backR += 0.1; else backR = 0;

        // logic for Sneaking 'vs' = variable sneak
        vs = Range.clip(vs, 0, 0.9 - 0.15*Math.abs(s/powerRatio) - 0.08*Math.abs(t/powerRatio));
        frontL = Range.clip(frontL, -1 + vs, 1 - vs);
        frontR = Range.clip(frontR, -1 + vs, 1 - vs);
        backL  = Range.clip(backL, -1 + vs, 1 - vs);
        backR  = Range.clip(backR, -1 + vs, 1 - vs);
            
        // set power for mecanum wheels
        frontLeft.setPower(frontL);
        frontRight.setPower(frontR);
        backLeft.setPower(backL);
        backRight.setPower(backR);
    }
    
    //gives encoder values for driver motors
    public void updateEncoderValues() {
        // odometry encoders
        encoderFL = ticksPerRotation;
    }

    //returns a power value based on
    public double powerRamp(double period, double startPow, double endPow) {
        if(rampTime.seconds() > period)
            return endPow;
        if(startPow > endPow)
            return Range.clip((rampTime.seconds() / period) * endPow, endPow, startPow);
        return Range.clip((rampTime.seconds() / period) * endPow, startPow, endPow);

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
