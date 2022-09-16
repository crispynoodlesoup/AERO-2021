package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeBlueLight())
                // Background opacity from 0-1
                .setBackgroundAlpha(0.8f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(240), Math.toRadians(240), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(180)))
                                .back(19)
                                .splineToConstantHeading(new Vector2d(38,-42), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(42,-38), 0)
                                .splineToSplineHeading(new Pose2d(55,-38, Math.toRadians(270)), 0)
                                .splineToConstantHeading(new Vector2d(66,-4), Math.toRadians(90))
                                .build()
                )
                .start();
    }
}