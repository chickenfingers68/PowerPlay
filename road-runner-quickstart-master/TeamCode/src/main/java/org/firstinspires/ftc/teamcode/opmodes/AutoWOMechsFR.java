package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoWOMechsFR")
 //@Disabled
public class AutoWOMechsFR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        com.acmerobotics.roadrunner.geometry.Pose2d pose1 = new Pose2d(-63, -12, Math.toRadians(180));

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-34, 0))
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .strafeLeft(4)
                //.splineToConstantHeading(new Vector2d(-34, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                .waitSeconds(.5)

                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                //.splineToSplineHeading(new Pose2d( -36, -12, Math.toRadians(225)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-63, -13, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(.5)

                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-27, -5, Math.toRadians(225)), Math.toRadians(45))
                //.splineToSplineHeading(new Pose2d( -36, -12, Math.toRadians(225)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-63, -14, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(.5)

                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(200)), Math.toRadians(45))
                //.splineToSplineHeading(new Pose2d( -36, -12, Math.toRadians(225)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-63, -16, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(.5)

                .build();
                

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(leftSeq);
    }
}
