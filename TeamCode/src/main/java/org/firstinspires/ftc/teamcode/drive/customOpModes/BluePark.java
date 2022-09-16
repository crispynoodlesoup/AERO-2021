package org.firstinspires.ftc.teamcode.drive.customOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous
public class BluePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //note, i should get ashar gobilda merch for christmas
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final double RIGHT_SERVO_UP   = 0.788;
        final double RIGHT_SERVO_DOWN = 0.368;
        final double LEFT_SERVO_UP    = 0;
        final double LEFT_SERVO_DOWN  = 0.362;

        final double RIGHT_INTAKE_POWER  = -0.6;
        final double RIGHT_OUTTAKE_POWER = 0.45;
        final double LEFT_INTAKE_POWER   = 0.6;
        final double LEFT_OUTTAKE_POWER  = -0.45;

        final double CLAMP_CLOSED  = 0.48;
        final double CLAMP_INTAKE  = 0.1;
        final double CLAMP_DEPOSIT = 0.3;

        final double DROP_PLATE_HOME = 0;
        final double DROP_PLATE_TOP  = 0.55;
        final double DROP_PLATE_MID  = 0.7;
        final double DROP_PLATE_BOT  = 0.82;

        Pose2d startPose = new Pose2d(-12, 65, 0);

        drive.setPoseEstimate(startPose);

        Trajectory trajPreloadTop = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    drive.dropPlate.setPosition(DROP_PLATE_TOP);
                })
                .strafeRight(18)
                .addDisplacementMarker(() -> {
                    drive.clamp.setPosition(CLAMP_INTAKE);
                })
                .build();

        Trajectory trajPreloadMid = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    drive.dropPlate.setPosition(DROP_PLATE_MID);
                })
                .strafeRight(18)
                .addDisplacementMarker(() -> {
                    drive.clamp.setPosition(CLAMP_DEPOSIT);
                })
                .build();

        Trajectory trajPreloadBot = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    drive.dropPlate.setPosition(DROP_PLATE_BOT);
                })
                .strafeRight(18)
                .addDisplacementMarker(() -> {
                    drive.clamp.setPosition(CLAMP_DEPOSIT);
                })
                .build();

        Trajectory trajCollectTop = drive.trajectoryBuilder(trajPreloadTop.end())
                .addTemporalMarker(0.5, () -> {
                    drive.dropPlate.setPosition(DROP_PLATE_HOME);
                })
                .addTemporalMarker(1, () -> {
                    drive.leftClaw.setPosition(LEFT_SERVO_DOWN);
                })
                .splineToConstantHeading(new Vector2d(12, 69), 0)
                .forward(35)
                .build();

        Trajectory trajCollectMid = drive.trajectoryBuilder(trajPreloadMid.end())
                .addTemporalMarker(0.5, () -> {
                    drive.dropPlate.setPosition(DROP_PLATE_HOME);
                    drive.clamp.setPosition(CLAMP_INTAKE);
                })
                .addTemporalMarker(1, () -> {
                    drive.leftClaw.setPosition(LEFT_SERVO_DOWN);
                })
                .splineToConstantHeading(new Vector2d(12, 69), 0)
                .forward(35)
                .build();

        Trajectory trajCollectBot = drive.trajectoryBuilder(trajPreloadBot.end())
                .addTemporalMarker(0.5, () -> {
                    drive.dropPlate.setPosition(DROP_PLATE_HOME);
                    drive.clamp.setPosition(CLAMP_INTAKE);
                })
                .addTemporalMarker(1, () -> {
                    drive.leftClaw.setPosition(LEFT_SERVO_DOWN);
                })
                .splineToConstantHeading(new Vector2d(12, 69), 0)
                .forward(35)
                .build();

        Trajectory trajStrafe = drive.trajectoryBuilder(trajCollectBot.end())
                .strafeRight(20)
                .build();

        drive.dropPlate.setPosition(DROP_PLATE_HOME);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajPreloadTop);
        drive.followTrajectory(trajCollectTop);
        drive.followTrajectory(trajStrafe);
    }
}
