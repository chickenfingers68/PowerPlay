package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Current {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 35, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-29.5, 2, Math.toRadians(175)), Math.toRadians(45))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-60.3, -12), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-30, 0 ), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-60.3, -12), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-30, 0 ), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-60.3, -12), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-30, 0 ), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineTo(new Vector2d(-32, -8))
                                .splineToConstantHeading(new Vector2d(-50, -8), Math.toRadians(180))


                                .build()

                );

        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 25, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(28.3, 1, Math.toRadians(5)), Math.toRadians(135))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(29, -4))
                                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(55, -12))
                                .splineToConstantHeading(new Vector2d(31.5, 4.75), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(29, -4))
                                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(55, -12))
                                .splineToConstantHeading(new Vector2d(31.5, 4.75), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(29, -4))
                                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(55, -12))
                                .splineToConstantHeading(new Vector2d(31.5, 4.75), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineTo(new Vector2d(32, -6))
                                .splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(0))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(left)
                .addEntity(right)
                .start();
    }
}
