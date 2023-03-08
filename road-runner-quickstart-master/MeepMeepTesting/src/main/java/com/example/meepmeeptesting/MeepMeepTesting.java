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
                                drive.trajectorySequenceBuilder(new Pose2d(28.625, -61, Math.toRadians(90)))
                                        .lineToConstantHeading(new Vector2d(35, -61))
                                        .waitSeconds(0.01)
                                        .splineToSplineHeading(new Pose2d(34, -24, Math.toRadians(0)), Math.toRadians(90))
                                        .waitSeconds(1.5)

                                        .lineToConstantHeading(new Vector2d(34, -20))
                                        .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(24, -12))
                                        .splineToConstantHeading(new Vector2d( 5, -24), Math.toRadians(180))
                                        .waitSeconds(1.5)

                                        .lineToConstantHeading(new Vector2d(5, -23))
                                        .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(24, -12))
                                        .splineToConstantHeading(new Vector2d( 5, -24), Math.toRadians(180))
                                        .waitSeconds(1.5)

                                        .lineToConstantHeading(new Vector2d(5, -23))
                                        .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(24, -12))
                                        .splineToConstantHeading(new Vector2d( 5, -24), Math.toRadians(180))
                                        .waitSeconds(1.5)

                                        //.splineToSplineHeading(new Pose2d(34, -16, Math.toRadians(0)), Math.toRadians(0))
                                        //.splineToSplineHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rMid)
                .start();
    }
}
