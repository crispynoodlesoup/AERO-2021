package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/* This class is responsible for managing all control loops and doing math
 * Control loops are loops that manage all the power output of the robot
 * We also have to take into account sensor data to make decisions,
 * meaning working with gyroscopes, encoders, odometry and the like
 */
public class ControlCalc {
    //define class members
    private HardwareMecanumbot myRobot;
    
    //variables
    double milliseconds = 1;
    double angleError = 0, angleErrorPrev = 0;
    double angleP, angleD, angleI;
    double turnP = 0;
    double driveSpeed = 0;
    double driveForwardAvg = 0;
    double driveTurnAvg = 0;
    
    public ControlCalc() { }
    
    // initialize new object and reset sensors
    public void initDrive(HardwareMecanumbot robot) {
        myRobot = robot;
        myRobot.resetAngle();
    }
    
    public void calcTurnPower(double target) {
        // calculate angular velocity using difference in angle and difference in time
        angleD = (angleError - angleErrorPrev) / (myRobot.runtime.milliseconds() - milliseconds);
        
        // calculate integral
        angleI += 0.00001 * angleError * (myRobot.runtime.milliseconds() - milliseconds);
        
        //update values
        milliseconds = myRobot.runtime.milliseconds();
        angleErrorPrev = angleError;
        angleError = target - myRobot.getAngle();
        
        // do an angle wrap so -180 < angleError < 180
        if(angleError < -180)
            angleError += 360;
        if(angleError > 180)
            angleError -= 360;
        
        // clipping values
        angleP = Range.clip(angleError/24.0, -1, 1);
        angleI = Range.clip(angleI, -Math.abs(0.2*angleP), Math.abs(0.2*angleP));
        angleD = Range.clip(angleD, -Math.abs(0.8*angleP), Math.abs(0.8*angleP));
        
        // PID coming together...
        // comments for PID tuning purposes
        turnP = angleP + 0.2*angleI + 4.5*angleD;
        //turnP = angleP;
        //turnP = angleP + 5*angleD;
        
        // clip turnP and create a deadzone
        turnP = Range.clip(turnP, -1, 1);
        if(Math.abs(turnP + driveSpeed) < 0.05)
            turnP = 0;
    }
    
    // calculates drivetrain power using target distance in wheel rotations
    public void calcDrivePower(double speed, double distance) {
        // update encoder values and calculate drive distance
        driveForwardAvg = (myRobot.encoderFL + myRobot.encoderFR + myRobot.encoderBL + myRobot.encoderBR) / 4.0;
        driveTurnAvg    =  myRobot.encoderFL - myRobot.encoderFR + myRobot.encoderBL - myRobot.encoderBR;
        
        // math to accelerate and decelerate at the beginning and end (trapezoidal profile)
        if(Math.abs(distance) > 2) {
            if(driveForwardAvg < 1 && driveForwardAvg > -1)
                driveSpeed = driveForwardAvg;
            else if(Math.abs(driveForwardAvg) > Math.abs(distance) - 1.0)
                driveSpeed = distance - driveForwardAvg;
            else
                driveSpeed = 1;
        } else {
            if(Math.abs(driveForwardAvg) < Math.abs(0.5*distance))
                driveSpeed = driveForwardAvg;
            else
                driveSpeed = distance - driveForwardAvg;
        }
        
        // clip and multiply by speed
        driveSpeed = Range.clip(speed * driveSpeed, -1, 1);
        
        // work on motion profiles and feedforward controls later, this is fine for now
    }
    
    public void move(double targetAng, double targetDis, double speed) {
        calcTurnPower(targetAng);
        calcDrivePower(speed, targetDis);
        //myRobot.moveMecanum(driveSpeed, 0, -turnP, 0);
    }
}
