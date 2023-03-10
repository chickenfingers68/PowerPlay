package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepTest2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 35, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-28, 1, Math.toRadians(175)), Math.toRadians(45))
                                .waitSeconds(1.5)

                                .splineToConstantHeading(new Vector2d(-45, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, -12), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToSplineHeading(new Pose2d(-24, -6, Math.toRadians(270)), Math.toRadians(0))
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, -12), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToSplineHeading(new Pose2d(-24, -6, Math.toRadians(270)), Math.toRadians(0))
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, -12), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToSplineHeading(new Pose2d(-24, -6, Math.toRadians(270)), Math.toRadians(0))
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, -12), Math.toRadians(180))


                                .build()

                );

        RoadRunnerBotEntity rn = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 25, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-29, 1, Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-29, 1), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-29, 1), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-29, 1), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineTo(new Vector2d(-32, -12))
                                .splineToConstantHeading(new Vector2d(-12, -12), Math.toRadians(0))
                                .build()

                );

        RoadRunnerBotEntity rightrn = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 25, Math.toRadians(180), Math.toRadians(60), 11.9)
                .setDimensions(12.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-29, 2, Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(29, -12))
                                .splineToConstantHeading(new Vector2d(59.5, -12), Math.toRadians(0))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(57, -12))
                                .splineToConstantHeading(new Vector2d(29, -2), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(50, -12))
                                .splineToConstantHeading(new Vector2d(59.5, -12), Math.toRadians(0))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(57, -12))
                                .splineToConstantHeading(new Vector2d(29, -2), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineToConstantHeading(new Vector2d(-45, -12))
                                .splineToConstantHeading(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(1.)
                                .lineToConstantHeading(new Vector2d(-57, -12))
                                .splineToConstantHeading(new Vector2d(-29, 1), Math.toRadians(90))
                                .waitSeconds(1.5)

                                .lineTo(new Vector2d(-32, -12))
                                .splineToConstantHeading(new Vector2d(-12, -12), Math.toRadians(0))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightrn)
                .start();
    }
}
