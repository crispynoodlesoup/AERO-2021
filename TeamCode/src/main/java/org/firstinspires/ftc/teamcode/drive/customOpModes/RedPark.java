package org.firstinspires.ftc.teamcode.drive.customOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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
public class RedPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {/*
        //note, i should get ashar gobilda merch for christmas
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-10, -62.5, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajPreload = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    drive.slides.setPower(0.8);
                })
                .addTemporalMarker(1.5, () -> {
                    drive.slides.setPower(0);
                    drive.dropPlate.setPosition(0.5);
                })
                .strafeRight(12)
                .waitSeconds(2)
                .build();

        Trajectory trajPark = drive.trajectoryBuilder(trajPreload.end(), true)
                .addTemporalMarker(0, () -> {
                    drive.slides.setPower(-0.6);
                    drive.dropPlate.setPosition(0);
                })
                .addTemporalMarker(2.5, () -> {
                    drive.slides.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(16,-64.5), 0, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(2),
                        new MecanumVelocityConstraint(15, 26)
                )), new ProfileAccelerationConstraint(15))
                .back(12)
                .build();

        Trajectory trajParkTwo = drive.trajectoryBuilder(trajPreload.end(), true)
                .splineTo(new Vector2d(36, -52), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(45,-40), 0)
                .splineToConstantHeading(new Vector2d(60, -40), 0)
                .build();

        drive.dropPlate.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajPreload);
        drive.followTrajectory(trajPark);
        drive.followTrajectory(trajParkTwo);*/

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, -65, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        final double RIGHT_SERVO_UP   = 0.788;
        final double RIGHT_SERVO_DOWN = 0.368;
        final double LEFT_SERVO_UP    = 0;
        final double LEFT_SERVO_DOWN  = 0.362;

        final double RIGHT_INTAKE_POWER  = -0.7;
        final double RIGHT_OUTTAKE_POWER = 0.45;
        final double LEFT_INTAKE_POWER   = 0.7;
        final double LEFT_OUTTAKE_POWER  = -0.45;

        final double CLAMP_CLOSED  = 0.48;
        final double CLAMP_INTAKE  = 0.1;
        final double CLAMP_DEPOSIT = 0.3;

        final double DROP_PLATE_HOME = 0;
        final double DROP_PLATE_TOP  = 0.5;
        final double DROP_PLATE_MID  = 0.7;
        final double DROP_PLATE_BOT  = 0.85;

        Trajectory trajPreload = drive.trajectoryBuilder(startPose, true)
                .addTemporalMarker(0, () -> {
                    drive.slides.setPower(0.8);
                })
                .addTemporalMarker(0.8, () -> {
                    drive.dropPlate.setPosition(0.5);
                    while(drive.slidePositionToPower(0.9) && opModeIsActive() && !isStopRequested()) {
                        telemetry.addData("slide position", drive.getSlidePosition());
                        telemetry.update();
                    }
                })
                .strafeRight(18)
                .build();

        Trajectory trajCollect = drive.trajectoryBuilder(trajPreload.end(), true)
                .addTemporalMarker(0.5, () -> {
                    drive.dropPlate.setPosition(0);
                    drive.slides.setPower(-0.1);
                })
                .addTemporalMarker(1.5, () -> {
                    drive.rightIntake.setPower(RIGHT_INTAKE_POWER);
                    drive.rightClaw.setPosition(RIGHT_SERVO_DOWN);
                })
                .addTemporalMarker(2.8, () -> {
                    while(drive.slidePositionToPower(0) && opModeIsActive() && !isStopRequested()) {
                        telemetry.addData("slide position", drive.getSlidePosition());
                        telemetry.update();
                    }
                })
                .addTemporalMarker(3.5, () -> {
                    drive.rightIntake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(12, -69), 0)
                .back(32)
                .build();

        Trajectory trajStrafe = drive.trajectoryBuilder(trajCollect.end())
                .strafeRight(24)
                .build();

        drive.dropPlate.setPosition(0);
        drive.rightClaw.setPosition(RIGHT_SERVO_UP);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajPreload);
        drive.followTrajectory(trajCollect);
        drive.followTrajectory(trajStrafe);
    }
}
