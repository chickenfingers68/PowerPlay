package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 38, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-32, 2, Math.toRadians(180)), Math.toRadians(90))
                                //.lineTo(new Vector2d(-34, 0))
                                //.turn(Math.toRadians(90))
                                .waitSeconds(1)

                                .strafeLeft(4)
                                //.splineToConstantHeading(new Vector2d(-34, -12), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .lineTo(new Vector2d(-51, -12))
                                .splineToConstantHeading(new Vector2d(-32, 2), Math.toRadians(90))
                                .waitSeconds(1)

                                .strafeLeft(4)
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .lineTo(new Vector2d(-51, -12))
                                .splineToConstantHeading(new Vector2d(-32, 2), Math.toRadians(90))
                                .waitSeconds(1)

                                .strafeLeft(4)
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .lineTo(new Vector2d(-51, -12))
                                .splineToConstantHeading(new Vector2d(-32, 2), Math.toRadians(90))
                                .waitSeconds(1)

                                .strafeLeft(4)
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .lineTo(new Vector2d(-51, -12))
                                .splineToConstantHeading(new Vector2d(-32, 2), Math.toRadians(90))
                                .waitSeconds(1)

                                .strafeLeft(4)
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(.5)

                                .build()

                );
        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 35, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-27., 2, Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-51, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-25.5, 3.5), Math.toRadians(90))

                                .build()

                );

        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 38, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(27., 2, Math.toRadians(0)), Math.toRadians(135))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(32, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(32, -12))
                                .lineToConstantHeading(new Vector2d(32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(32, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(32, -12))
                                .lineToConstantHeading(new Vector2d(32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(32, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(32, -12))
                                .lineToConstantHeading(new Vector2d(32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(32, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                .build()
                );
/*
        RoadRunnerBotEntity lMid = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 51, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                        //.splineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(90))
                                        .lineToSplineHeading(new Pose2d(-34, -24, Math.toRadians(180)))
                                        .waitSeconds(1)
                                        .strafeRight(4)
                                        //.splineToConstantHeading(new Vector2d(-34, -12), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                        .waitSeconds(.5)

                                //.lineTo(new Vector2d(-45, -12))
                                //.splineToSplineHeading(new Pose2d(-30, -6, Math.toRadians(225)), Math.toRadians(45))
                                //.waitSeconds(1)
                                //.splineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)), Math.toRadians(180))

                                        .lineTo(new Vector2d(-30, -12))
                                        .splineToSplineHeading(new Pose2d( -6, -18, Math.toRadians(135)), Math.toRadians(-45))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(-34, -16, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(.5)

                                        .lineTo(new Vector2d(-30, -12))
                                        .splineToSplineHeading(new Pose2d( -6, -18, Math.toRadians(135)), Math.toRadians(-45))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(-34, -16, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(.5)

                                        .lineTo(new Vector2d(-30, -12))
                                        .splineToSplineHeading(new Pose2d( -6, -18, Math.toRadians(135)), Math.toRadians(-45))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(-34, -16, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(.5)

                                        .lineTo(new Vector2d(-30, -12))
                                        .splineToSplineHeading(new Pose2d( -6, -18, Math.toRadians(135)), Math.toRadians(-45))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(-34, -16, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(.5)

                                        .build()
                );

        RoadRunnerBotEntity rMid = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 51, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, -61, Math.toRadians(90)))
                                        //.splineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(90))
                                        .lineToSplineHeading(new Pose2d(34, -24, Math.toRadians(0)))
                                        .waitSeconds(1)
                                        .strafeLeft(4)
                                        //.splineToConstantHeading(new Vector2d(-34, -12), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                        .waitSeconds(.5)

                                //.lineTo(new Vector2d(-45, -12))
                                //.splineToSplineHeading(new Pose2d(-30, -6, Math.toRadians(225)), Math.toRadians(45))
                                //.waitSeconds(1)
                                //.splineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)), Math.toRadians(180))

                                        .lineTo(new Vector2d(30, -12))
                                        .splineToSplineHeading(new Pose2d( 6, -18, Math.toRadians(45)), Math.toRadians(-135))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(34, -16, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))
                                        .waitSeconds(.5)

                                        .lineTo(new Vector2d(30, -12))
                                        .splineToSplineHeading(new Pose2d( 6, -18, Math.toRadians(45)), Math.toRadians(-135))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(34, -16, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))
                                        .waitSeconds(.5)

                                        .lineTo(new Vector2d(30, -12))
                                        .splineToSplineHeading(new Pose2d( 6, -18, Math.toRadians(45)), Math.toRadians(-135))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(34, -16, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))
                                        .waitSeconds(.5)

                                        .lineTo(new Vector2d(30, -12))
                                        .splineToSplineHeading(new Pose2d( 6, -18, Math.toRadians(45)), Math.toRadians(-135))
                                        .waitSeconds(1)
                                        .splineToSplineHeading(new Pose2d(34, -16, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))

                                        .build()
                );
        */
        RoadRunnerBotEntity test2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 38, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-27., 2, Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-32, -12))
                                .lineToConstantHeading(new Vector2d(-32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-32, -12))
                                .lineToConstantHeading(new Vector2d(-32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-32, -12))
                                .lineToConstantHeading(new Vector2d(-32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-32, -12))
                                .lineToConstantHeading(new Vector2d(-32, 2))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(right)
                .start();
    }
}
