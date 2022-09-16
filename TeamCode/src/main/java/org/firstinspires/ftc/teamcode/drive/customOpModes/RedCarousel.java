package org.firstinspires.ftc.teamcode.drive.customOpModes;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous
public class RedCarousel extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "model_20220225_095344.tflite";
    private static final String[] LABELS = {
            "aero_tse"
    };

    private static final String VUFORIA_KEY =
            "AeSUQOr/////AAABmYfucJklj0uZh64olOY2GRx+gCxOH5qAXZ5c+qY6jQT6i7UaQ1PK68H1yacgOvEdWZ1wCPdm4ztImT10prBKUVp/pYMG2oPaV2/1hqLS+Q9fuH17GvuJL8TmUtHBk3arMCbzgXNIoaBTyYsuW1nE/2nMxq78KASNQYh6Co3UwXVSYk3XkNbZ2UiRL1ZAPDEM/k/TU4tWi1c059L9sCjp7eaU9A4Nn4yKeg2CPASEBy7E4rVGo+yPMd6kHF2nwdmyfcmxtcjGbAmmw85ippllc1CWbc4E2m6RAwcNnEFVP7c9D2a3LRmJQB+Tpi8GE54sVJuIj8j3iZYZExI+w6Q1sp8JoMwDG3ROQpjhAOJjCVtf";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        FtcDashboard.getInstance().startCameraStream(tfod, 0);

        //note, i should get ashar gobilda merch for christmas
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-52, -65, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajWait = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    while(drive.slidePositionToPower(2) && opModeIsActive()) {}
                    drive.dropPlate.setPosition(0.6);
                })
                .waitSeconds(20)
                .build();

        TrajectorySequence trajCarousel = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(3)
                .splineToConstantHeading(new Vector2d(-60,-59), Math.toRadians(180))
                .addTemporalMarker(1.8, () -> {
                    drive.carousel.setPower(0.2);
                })
                .addTemporalMarker(2.4, () -> {
                    drive.carousel.setPower(0.4);
                })
                .addTemporalMarker(3, () -> {
                    drive.carousel.setPower(0.6);
                })
                .addTemporalMarker(3.6, () -> {
                    drive.carousel.setPower(0.8);
                })
                .addTemporalMarker(4, () -> {
                    drive.carousel.setPower(0);
                })
                .waitSeconds(3)
                .build();

        Trajectory trajPreload = drive.trajectoryBuilder(trajCarousel.end())
                .addTemporalMarker(0.2, () -> {
                    drive.slides.setPower(-0.4);
                })
                .addTemporalMarker(1.5, () -> {
                    drive.slides.setPower(-0.01);
                })
                .addTemporalMarker(3, () -> {
                    drive.dropPlate.setPosition(0.65);
                })
                .splineTo(new Vector2d(-44, -24), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    while(drive.slidePositionToPower(1.1) && opModeIsActive()) {}
                    drive.clamp.setPosition(0.25);
                })
                .build();

        Trajectory trajWarehousePark = drive.trajectoryBuilder(trajPreload.end())
                .addTemporalMarker(0.2, () -> {
                    drive.dropPlate.setPosition(0.05);
                })
                .splineToConstantHeading(new Vector2d(-70, -39), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    while(drive.slidePositionToPower(0) && opModeIsActive()) {}
                })
                .build();

        drive.dropPlate.setPosition(0);
        drive.rightClaw.setPosition(0.788);
        drive.clamp.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) return;

        //drive.followTrajectorySequence(trajWait);
        drive.followTrajectorySequence(trajCarousel);
        drive.followTrajectory(trajPreload);
        drive.followTrajectory(trajWarehousePark);
        //drive.rightClaw.setPosition();
        //drive.followTrajectory(trajPark);
        //drive.followTrajectory(trajParkTwo);

        /*
        //note, i should get ashar gobilda merch for christmas
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-52, -65, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajWait = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    while(drive.slidePositionToPower(2) && opModeIsActive()) {}
                    drive.dropPlate.setPosition(0.5);
                })
                .waitSeconds(20)
                .build();

        TrajectorySequence trajCarousel = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(-60,-57), Math.toRadians(180))
                .addTemporalMarker(1.8, () -> {
                    drive.carousel.setPower(0.2);
                })
                .addTemporalMarker(2.2, () -> {
                    drive.carousel.setPower(0.4);
                })
                .addTemporalMarker(2.6, () -> {
                    drive.carousel.setPower(0.6);
                })
                .addTemporalMarker(3, () -> {
                    drive.carousel.setPower(0.8);
                })
                .addTemporalMarker(3.6, () -> {
                    drive.carousel.setPower(0);
                })
                .waitSeconds(1.8)
                .resetVelConstraint()
                .resetAccelConstraint()
                .build();

        Trajectory trajPreload = drive.trajectoryBuilder(trajCarousel.end())
                .addTemporalMarker(0.2, () -> {
                    drive.slides.setPower(0.5);
                })
                .addTemporalMarker(1.5, () -> {
                    drive.slides.setPower(0.02);
                })
                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    while(drive.slidePositionToPower(0.55) && opModeIsActive()) {}
                    drive.dropPlate.setPosition(0.6);
                })
                .build();

        Trajectory trajWarehousePark = drive.trajectoryBuilder(trajPreload.end())
                .splineToConstantHeading(new Vector2d(-70, -36), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    drive.dropPlate.setPosition(0.05);
                    while(drive.slidePositionToPower(0) && opModeIsActive()) {}
                })
                .build();

        drive.dropPlate.setPosition(0);
        drive.rightClaw.setPosition(0.788);

        waitForStart();

        if (isStopRequested()) return;

        //drive.followTrajectorySequence(trajWait);
        drive.followTrajectorySequence(trajCarousel);
        drive.followTrajectory(trajPreload);
        drive.followTrajectory(trajWarehousePark);
        //drive.rightClaw.setPosition();
        //drive.followTrajectory(trajPark);
        //drive.followTrajectory(trajParkTwo);

        */
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
