package org.firstinspires.ftc.teamcode.drive.customOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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
public class BlueCarousel extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "model_20220225_095344.tflite";
    private static final String[] LABELS = {
            "aero_tse"
    };

    private static final String VUFORIA_KEY =
            "AeSUQOr/////AAABmYfucJklj0uZh64olOY2GRx+gCxOH5qAXZ5c+qY6jQT6i7UaQ1PK68H1yacgOvEdWZ1wCPdm4ztImT10prBKUVp/pYMG2oPaV2/1hqLS+Q9fuH17GvuJL8TmUtHBk3arMCbzgXNIoaBTyYsuW1nE/2nMxq78KASNQYh6Co3UwXVSYk3XkNbZ2UiRL1ZAPDEM/k/TU4tWi1c059L9sCjp7eaU9A4Nn4yKeg2CPASEBy7E4rVGo+yPMd6kHF2nwdmyfcmxtcjGbAmmw85ippllc1CWbc4E2m6RAwcNnEFVP7c9D2a3LRmJQB+Tpi8GE54sVJuIj8j3iZYZExI+w6Q1sp8JoMwDG3ROQpjhAOJjCVtf";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime timer = new ElapsedTime() ;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        FtcDashboard.getInstance().startCameraStream(tfod, 0);

        /*

        //note, i should get ashar gobilda merch for christmas
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, 63, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajCarousel = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.01, () -> {
                    drive.slides.setPower(0.6);
                })
                .addTemporalMarker(1.75, () -> {
                    drive.slides.setPower(0.02);
                })
                .splineToConstantHeading(new Vector2d(-52,57.5), Math.toRadians(180))
                .turn(Math.toRadians(90))
                .strafeLeft(6)
                //.splineToConstantHeading(new Vector2d(-56,-58), Math.toRadians(180))
                .addTemporalMarker(2.8, () -> {
                    drive.carousel.setPower(-0.2);
                })
                .addTemporalMarker(3.2, () -> {
                    drive.carousel.setPower(-0.4);
                })
                .addTemporalMarker(3.6, () -> {
                    drive.carousel.setPower(-0.6);
                })
                .addTemporalMarker(4, () -> {
                    drive.carousel.setPower(-0.8);
                })
                .addTemporalMarker(5, () -> {
                    drive.carousel.setPower(0);
                })
                .waitSeconds(3)
                .build();

        Trajectory trajPreloadTop = drive.trajectoryBuilder(trajCarousel.end(), true)
                .splineToConstantHeading(new Vector2d(-31, 20), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    drive.dropPlate.setPosition(0.5);
                    drive.clamp.setPosition(0.2);
                })
                .build();

        Trajectory trajPreloadMid = drive.trajectoryBuilder(trajCarousel.end(), true)
                .splineToConstantHeading(new Vector2d(-31, 20), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    drive.dropPlate.setPosition(0.7);
                    drive.clamp.setPosition(0.3);
                })
                .build();

        Trajectory trajPreloadBot = drive.trajectoryBuilder(trajCarousel.end(), true)
                .splineToConstantHeading(new Vector2d(-31, 20), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    drive.dropPlate.setPosition(0.85);
                    drive.clamp.setPosition(0);
                })
                .build();

        Trajectory trajWarehousePark = drive.trajectoryBuilder(trajPreloadTop.end())
                .addTemporalMarker(0.2, () -> {
                    drive.dropPlate.setPosition(0.05);
                    drive.slides.setPower(-0.5);
                })
                .addTemporalMarker(1, () -> {
                    drive.slides.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-60, 37), 0)
                .build();

        drive.dropPlate.setPosition(0.05);

        double capPosition = -1;

        telemetry.addData("ready for initialization", 0);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();
        while(timer.seconds() < 5) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if(updatedRecognitions.size() == 0)
                        capPosition = -1;
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        capPosition = recognition.getLeft();
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                    telemetry.update();
                }
            }
        }

        drive.followTrajectorySequence(trajCarousel);
        if(capPosition == -1)
            drive.followTrajectory(trajPreloadBot);
        else if(capPosition < 300)
            drive.followTrajectory(trajPreloadMid);
        else
            drive.followTrajectory(trajPreloadTop);
        drive.followTrajectory(trajWarehousePark);
        //drive.followTrajectory(trajPark);
        //drive.followTrajectory(trajParkOnePointFive);
        //drive.followTrajectory(trajParkTwo);
*/
        //note, i should get ashar gobilda merch for christmas
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, 62.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajCarousel = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.01, () -> {
                    drive.slides.setPower(-0.3);
                })
                .addTemporalMarker(1.75, () -> {
                    drive.slides.setPower(-0.01);
                })
                .back(5)
                .splineToConstantHeading(new Vector2d(-52,57.5), Math.toRadians(90))
                .strafeLeft(6)
                //.splineToConstantHeading(new Vector2d(-56,-58), Math.toRadians(180))
                .addTemporalMarker(2.8, () -> {
                    drive.carousel.setPower(-0.2);
                })
                .addTemporalMarker(3.2, () -> {
                    drive.carousel.setPower(-0.4);
                })
                .addTemporalMarker(3.6, () -> {
                    drive.carousel.setPower(-0.5);
                })
                .addTemporalMarker(4.5, () -> {
                    drive.carousel.setPower(-0.6);
                })
                .addTemporalMarker(5.6, () -> {
                    drive.carousel.setPower(0);
                })
                .waitSeconds(3)
                .build();

        Trajectory trajPreload = drive.trajectoryBuilder(trajCarousel.end(), true)
                .addTemporalMarker(0.1, () -> {
                    drive.dropPlate.setPosition(0.52);
                })
                .splineToConstantHeading(new Vector2d(-30.5, 18.5), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    drive.clamp.setPosition(0.22);
                })
                .build();

        Trajectory trajWarehousePark = drive.trajectoryBuilder(trajPreload.end())
                .addTemporalMarker(0.5, () -> {
                    drive.dropPlate.setPosition(0.05);
                    drive.slides.setPower(0.2);
                })
                .addTemporalMarker(1, () -> {
                    drive.slides.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-60, 36), 0)
                .build();

        Trajectory trajPark = drive.trajectoryBuilder(trajPreload.end())
                .addTemporalMarker(0, () -> {
                    drive.slides.setPower(0.25);
                })
                .addTemporalMarker(0.8, () -> {
                    drive.dropPlate.setPosition(0.05);
                    drive.slides.setPower(0);
                })
                .splineTo(new Vector2d(-16, 26), 0)
                .forward(16)
                .splineToConstantHeading(new Vector2d(16, 66.6), 0, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(2),
                        new MecanumVelocityConstraint(15, 26)
                )), new ProfileAccelerationConstraint(15))
                .forward(22)
                .build();

        drive.dropPlate.setPosition(0.05);
        drive.clamp.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajCarousel);
        drive.followTrajectory(trajPreload);
        drive.followTrajectory(trajWarehousePark);
        //drive.followTrajectory(trajPark);
        //drive.followTrajectory(trajParkOnePointFive);
        //drive.followTrajectory(trajParkTwo);

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
