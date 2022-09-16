 package org.firstinspires.ftc.teamcode.drive.customOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

 /*
  * This is a simple routine to test turning capabilities.
  */
 @Config
 @Autonomous
 public class BlueSharedHub extends LinearOpMode {
     @Override
     public void runOpMode() throws InterruptedException {
         //note, i should get ashar gobilda merch for christmas
         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         Pose2d startPose = new Pose2d(12, 65, 0);

         drive.setPoseEstimate(startPose);

         final double RIGHT_SERVO_UP   = 0.788;
         final double RIGHT_SERVO_DOWN = 0.368;
         final double LEFT_SERVO_UP    = 0;
         final double LEFT_SERVO_DOWN  = 0.362;

         final double RIGHT_INTAKE_POWER  = -0.6;
         final double RIGHT_OUTTAKE_POWER = 0.5;
         final double LEFT_INTAKE_POWER   = 0.6;
         final double LEFT_OUTTAKE_POWER  = -0.45;

         final double CLAMP_CLOSED  = 0.48;
         final double CLAMP_INTAKE  = 0.1;
         final double CLAMP_DEPOSIT = 0.3;

         final double DROP_PLATE_HOME = 0;
         final double DROP_PLATE_TOP  = 0.55;
         final double DROP_PLATE_MID  = 0.7;
         final double DROP_PLATE_BOT  = 0.82;

         Trajectory trajPreload = drive.trajectoryBuilder(startPose)
                 .addTemporalMarker(5.2, () -> {
                     drive.dropPlate.setPosition(DROP_PLATE_BOT);
                 })
                 .addTemporalMarker(6.5, () -> {
                     drive.clamp.setPosition(CLAMP_DEPOSIT);
                 })
                 .addTemporalMarker(7, () -> {
                     drive.dropPlate.setPosition(DROP_PLATE_HOME);
                     drive.clamp.setPosition(CLAMP_INTAKE);
                     drive.rightClaw.setPosition(RIGHT_SERVO_DOWN);
                 })
                 .forward(19)
                 .splineToConstantHeading(new Vector2d(38, 45), Math.toRadians(270))
                 .splineToConstantHeading(new Vector2d(42, 41), 0)
                 .splineToSplineHeading(new Pose2d(62, 40, Math.toRadians(270)), 0)
                 .splineToConstantHeading(new Vector2d(66, 6), Math.toRadians(270))
                 .build();

         Trajectory trajCollect = drive.trajectoryBuilder(trajPreload.end())
                 .addTemporalMarker(0.5, () -> {
                     drive.rightIntake.setPower(RIGHT_INTAKE_POWER);
                 })
                 .back(40)
                 .addDisplacementMarker(() -> {
                     drive.rightIntake.setPower(0);
                     drive.rightClaw.setPosition(RIGHT_SERVO_UP);
                 })
                 .build();

         Trajectory trajDeliver = drive.trajectoryBuilder(trajCollect.end(), true)
                 .addTemporalMarker(0.2, () -> {
                     drive.rightIntake.setPower(RIGHT_OUTTAKE_POWER);
                 })
                 .addTemporalMarker(1.5, () -> {
                     drive.clamp.setPosition(CLAMP_CLOSED);
                     drive.dropPlate.setPosition(DROP_PLATE_BOT);
                     drive.rightIntake.setPower(0);
                 })
                 .addTemporalMarker(2.7, () -> {
                     drive.clamp.setPosition(CLAMP_DEPOSIT);
                     drive.rightClaw.setPosition(RIGHT_SERVO_DOWN);
                 })
                 .forward(40)
                 .build();

         Trajectory trajCollect2 = drive.trajectoryBuilder(trajDeliver.end())
                 .addTemporalMarker(0.5, () -> {
                     drive.rightIntake.setPower(RIGHT_INTAKE_POWER);
                     drive.clamp.setPosition(CLAMP_INTAKE);
                     drive.dropPlate.setPosition(DROP_PLATE_HOME);
                 })
                 .back(44)
                 .addDisplacementMarker(() -> {
                     drive.rightIntake.setPower(0);
                     drive.rightClaw.setPosition(RIGHT_SERVO_UP);
                 })
                 .build();

         Trajectory trajDeliver2 = drive.trajectoryBuilder(trajCollect2.end(), true)
                 .addTemporalMarker(0.2, () -> {
                     drive.rightIntake.setPower(RIGHT_OUTTAKE_POWER);
                 })
                 .addTemporalMarker(1.5, () -> {
                     drive.clamp.setPosition(CLAMP_CLOSED);
                     drive.dropPlate.setPosition(DROP_PLATE_BOT);
                     drive.rightIntake.setPower(0);
                 })
                 .addTemporalMarker(2.7, () -> {
                     drive.clamp.setPosition(CLAMP_DEPOSIT);
                     drive.rightClaw.setPosition(RIGHT_SERVO_DOWN);
                 })
                 .forward(44)
                 .build();

         Trajectory trajCollect3 = drive.trajectoryBuilder(trajDeliver2.end())
                 .addTemporalMarker(0.5, () -> {
                     drive.rightIntake.setPower(RIGHT_INTAKE_POWER);
                     drive.clamp.setPosition(CLAMP_INTAKE);
                     drive.dropPlate.setPosition(DROP_PLATE_HOME);
                 })
                 .back(48)
                 .addDisplacementMarker(() -> {
                     drive.rightIntake.setPower(0);
                     drive.rightClaw.setPosition(RIGHT_SERVO_UP);
                 })
                 .build();

         Trajectory trajDeliver3 = drive.trajectoryBuilder(trajCollect3.end(), true)
                 .addTemporalMarker(0.2, () -> {
                     drive.rightIntake.setPower(RIGHT_OUTTAKE_POWER);
                 })
                 .addTemporalMarker(1.5, () -> {
                     drive.clamp.setPosition(CLAMP_CLOSED);
                     drive.dropPlate.setPosition(DROP_PLATE_BOT);
                     drive.rightIntake.setPower(0);
                 })
                 .addTemporalMarker(2.7, () -> {
                     drive.clamp.setPosition(CLAMP_DEPOSIT);
                     drive.rightClaw.setPosition(RIGHT_SERVO_DOWN);
                 })
                 .forward(50)
                 .build();

         Trajectory trajCollect4 = drive.trajectoryBuilder(trajDeliver3.end())
                 .addTemporalMarker(0.5, () -> {
                     drive.rightIntake.setPower(RIGHT_INTAKE_POWER);
                     drive.clamp.setPosition(CLAMP_INTAKE);
                     drive.dropPlate.setPosition(DROP_PLATE_HOME);
                 })
                 .back(54)
                 .addDisplacementMarker(() -> {
                     drive.rightIntake.setPower(0);
                     drive.rightClaw.setPosition(RIGHT_SERVO_UP);
                 })
                 .build();

         drive.dropPlate.setPosition(DROP_PLATE_HOME);
         drive.rightClaw.setPosition(RIGHT_SERVO_UP);
         drive.rightClaw.setPosition(RIGHT_SERVO_UP);
         drive.clamp.setPosition(CLAMP_CLOSED);

         waitForStart();

         if (isStopRequested()) return;

         drive.followTrajectory(trajPreload);
         drive.followTrajectory(trajCollect);
         drive.followTrajectory(trajDeliver);
         drive.followTrajectory(trajCollect2);
         drive.followTrajectory(trajDeliver2);
         drive.followTrajectory(trajCollect3);
         drive.followTrajectory(trajDeliver3);
         drive.followTrajectory(trajCollect4);

     }
 }
