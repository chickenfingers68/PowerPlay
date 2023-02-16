package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoWOMechsFR")
 @Disabled
public class AutoWOMechsFR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(startPose)

                .forward(36)
                .turn(Math.toRadians(-90))
                .forward(5)
                .back(5)
                /*
                .strafeLeft(11)
                .forward(4)
                .back(4)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -13, Math.toRadians(90)))
                .forward(5)
                .back(5)
                .strafeLeft(33)*/
                .build();

        TrajectorySequence midSeq = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(22.5)
                .forward(48)
                .strafeLeft(11)
                .forward(4)
                .back(4)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -13, Math.toRadians(90)))
                .forward(5)
                .back(5)
                .strafeLeft(12)
                .build();
        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(22.5)
                .forward(48)
                .strafeLeft(11)
                .forward(4)
                .back(4)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -13, Math.toRadians(90)))
                .forward(5)
                .back(5)
                .strafeRight(13)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(leftSeq);
    }
}
