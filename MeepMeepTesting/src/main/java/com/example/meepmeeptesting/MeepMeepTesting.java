package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    static double originToWall = 141.0/2.0; // I guess the field is actually 141 inches wide
    static double wallDistance = originToWall - 6.5; // Center of bot is 6.5in from wall

    static Pose2d startPos = new Pose2d(11.4,-(originToWall-9), Math.toRadians(-90));
    static Pose2d preloadDepositPos = new Pose2d(-2,-42,Math.toRadians(-60));

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 700 pixels at 50 fps
        MeepMeep meepMeep = new MeepMeep(740,50);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.6,18) // Width of 11.6 to match our thin bot
                .setConstraints(55, 30, Math.toRadians(167), Math.toRadians(167), 11)

                // The path we are simulating
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(preloadDepositPos)
                                .lineToSplineHeading(new Pose2d(2,-55, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(32,-63),0)// Move through gap
                                // Move toward shared hub, this one is slowed down to prevent slipping
                                .splineToConstantHeading(new Vector2d(37,-44), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(120),11))
                                .splineToSplineHeading(new Pose2d(63,-37, Math.toRadians(-90)), Math.toRadians(0)) // Park
                                .build()
                );
                meepMeep
                        .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                        // Set theme
                        .setTheme(new ColorSchemeRedDark())
                        // Background opacity from 0-1
                        .setBackgroundAlpha(0.9f)
                        .addEntity(bot)
                        .start();
    }
}