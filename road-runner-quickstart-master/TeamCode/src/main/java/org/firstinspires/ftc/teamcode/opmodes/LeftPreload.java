package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(name = "LeftPreload", group = "adv")
@Disabled
public class LeftPreload extends LinearOpMode {

    private Servo claw;
    public static double firstX = 27.5;
    double OPEN_CLAW = 0.78;
    double CLOSED_CLAW = 1.0;

    //Cameras
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.266;

    //6:1
    //16:2
    //12:3
    int LeftTag = 6; // Tag ID 18 from the 36h11 family
    int RightTag = 12;
    int MiddleTag = 16;
    int location = 0;

    AprilTagDetection tagOfInterest = null;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineToSplineHeading() trajectory

        WAIT_1,         // Then we're gonna wait a second to score
        TRAJECTORY_2,   // Then, strafeLeft()

        TRAJECTORY_3,   // Then, we follow a splineToConstantHeading() trajectory

        WAIT_2,         // Then we're gonna wait a second to pick up a cone
        TRAJECTORY_4,   // Then we want to move backwards a bit

        TRAJECTORY_5,   // Then, we follow another splineToConstantHeading() trajectory

        WAIT_3,         // Then we're gonna wait a second to score
        TRAJECTORY_6,
        TRAJECTORY_7,
        TRAJECTORY_2a,
        TRAJECTORY_5a,
        WAIT_4,
        WAIT_5,
        WAIT_6,
        WAIT_1a,

        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(90));
    Pose2d firstDropPose = new Pose2d(-28.6, 2, Math.toRadians(180));
    int counter = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLOSED_CLAW);
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(0.5, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    ////arm.setPower(-0.05)
                    claw.setPosition(CLOSED_CLAW);
                })
                .lineToLinearHeading(new Pose2d(-30.5, 0, Math.toRadians(176)))
                .addDisplacementMarker(25, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                    arm.setPower(-.3);
                })
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(-firstX, 0, Math.toRadians(180)))
                .lineTo(new Vector2d(-32, -16))
                .splineToConstantHeading(new Vector2d(-50, -12), Math.toRadians(180))
                .build();

        TrajectorySequence midPark = drive.trajectorySequenceBuilder(new Pose2d(-firstX, 0, Math.toRadians(180)))
                .lineTo(new Vector2d(-32, -16))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-firstX, 0, Math.toRadians(180)))
                .lineTo(new Vector2d(-32, -16))
                .splineToConstantHeading(new Vector2d(-16, -12), Math.toRadians(180))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime waitTimer1 = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LeftTag || tag.id == RightTag || tag.id == MiddleTag) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            /* Actually do something useful */
            if (tagOfInterest == null || tagOfInterest.id == LeftTag) {
                location = 1;
                telemetry.addLine("LEFT");
            } else if (tagOfInterest.id == RightTag) {
                location = 3;
                telemetry.addLine("Right");
            } else {
                location = 2;
                telemetry.addLine("Middle");
            }

            telemetry.update();
            sleep(20);
        }
        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        drive.setPoseEstimate(new Pose2d(new Vector2d(-firstX, -24), Math.toRadians(180)));
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_1:
                    counter++;
                    if(timer.seconds() > 0.5){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() > 0.55){
                        arm.setPower(.75);
                    }
                    if(timer.seconds() > 0.95){
                        lift.setTarget(100);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on
                    if (waitTimer1.seconds() >= 1) {
                        currentState = State.IDLE;
                        if(location == 1){
                            drive.followTrajectorySequenceAsync(leftPark);
                        } else if(location == 2){
                            drive.followTrajectorySequenceAsync(midPark);
                        } else if(location == 3){
                            drive.followTrajectorySequenceAsync(rightPark);
                        }
                    }
                    break;


                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    //arm.setPower(0);
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            arm.update();
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("State: ", currentState);
            telemetry.addData("target: ", lift.getTarget());
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}