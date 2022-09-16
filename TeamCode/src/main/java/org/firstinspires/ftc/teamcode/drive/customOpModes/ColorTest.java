package org.firstinspires.ftc.teamcode.drive.customOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory trajReturn = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive()) {
            //drive.colorSenseLeft();

            // Right Side Color Sensor
            if(drive.colorRight.green() > 500) {
                if(drive.colorRight.blue() * 1.5 < drive.colorRight.green())
                    telemetry.addData("Right Side Freight", "Block");
                else
                    telemetry.addData("Right Side Freight", "Ball");
            } else
                telemetry.addData("Right Side Freight", "None");

            // Left Side Color Sensor
            if(drive.colorLeft.green() > 500) {
                if(drive.colorLeft.blue() * 1.5 < drive.colorLeft.green())
                    telemetry.addData("Left Side Freight", "Block");
                else
                    telemetry.addData("Left Side Freight", "Ball");
            } else
                telemetry.addData("Left Side Freight", "None");

            telemetry.addData("right green", drive.colorRight.green());
            telemetry.addData("left green", drive.colorLeft.green());

            telemetry.addData("right blue", drive.colorRight.blue());
            telemetry.addData("left blue", drive.colorLeft.blue());
            telemetry.update();
        }

    }
}
