package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/* This is a helper class that basically initializes the hardware and
 * gives us functions to control the power of each motor and servo
 */
public class HardwareMecanumbot {//access instruments of Hub
    ElapsedTime runtime  = new ElapsedTime();
    BNO055IMU imu;
    Orientation angle;
    
    // motor declarations
    public DcMotor frontLeft  = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft   = null;
    public DcMotor backRight  = null;
    
    //variables
    double frontL, frontR, backL, backR;
    double ticksPerRotation;
    public double encoderFL, encoderFR, encoderBL, encoderBR;

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
        
        // initialize the IMU for gyroscope and accelerometer
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Encoders for odometry
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //encoders for driving
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //brake the motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
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
        frontL *= Math.abs(frontL)*0.8;
        frontR *= Math.abs(frontR)*0.8;
        backL  *= Math.abs(backL)*0.8;
        backR  *= Math.abs(backR)*0.8;
        if(frontL < -0.002) frontL -= 0.2; else if(frontL > 0.002) frontL += 0.2; else frontL = 0;
        if(frontR < -0.002) frontR -= 0.2; else if(frontR > 0.002) frontR += 0.2; else frontR = 0;
        if(backL < -0.002) backL -= 0.2; else if(backL > 0.002) backL += 0.2; else backL = 0;
        if(backR < -0.002) backR -= 0.2; else if(backR > 0.002) backR += 0.2; else backR = 0;

        // logic for Sneaking 'vs' = variable sneak
        vs = Range.clip(vs, 0, 0.8 - 0.1*s/powerRatio);
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
        // drive encode positions translated to inches using the equation:
        // revolutions * pi * wheel diameter
        encoderFL = (frontLeft.getCurrentPosition() / ticksPerRotation)  * 3.14159 * 3.937;
        encoderFR = (frontRight.getCurrentPosition() / ticksPerRotation) * 3.14159 * 3.937;
        encoderBL = (backLeft.getCurrentPosition() / ticksPerRotation)   * 3.14159 * 3.937;
        encoderBR = (backRight.getCurrentPosition() / ticksPerRotation)  * 3.14159 * 3.937;
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


