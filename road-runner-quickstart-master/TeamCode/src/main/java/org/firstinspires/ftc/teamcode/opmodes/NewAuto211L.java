package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
@Autonomous(name = "i hate autonomous left", group = "adv")
@Disabled
public class NewAuto211L extends LinearOpMode {

    private Servo claw;
    public static double firstX = 27.5;
    double OPEN_CLAW = 0.78;
    double CLOSED_CLAW = 1.0;
    boolean r = true;
    boolean a = false;
    boolean b = false;
    boolean c = false;
    boolean d = false;
    boolean e = false;
    boolean f = false;
    boolean g = false;
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;
    double target, _correction;
    private IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .5, correction;

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
        TRAJECTORY_2b,
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
    ElapsedTime runtime = new ElapsedTime();
    int counter = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //Constant Variables
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        //SetWheelsToZero
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderFunction();
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
                .lineToLinearHeading(new Pose2d(-30.5, -20, Math.toRadians(176)))
                .addDisplacementMarker(25, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(1400);
                    arm.setPower(-.3);
                })
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(-firstX, -24, Math.toRadians(180)))
                .addTemporalMarker(0.25, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.15);
                    claw.setPosition(OPEN_CLAW);
                })
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.idle();
                    claw.setPosition(OPEN_CLAW);
                })
                .lineToConstantHeading(new Vector2d(-31, -11.5))
                .build();

        Trajectory trajectory2a = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(-57.5, -9.9, Math.toRadians(171)))
                .addDisplacementMarker(15, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.idle();
                    //todo
                    arm.setPower(0.6);
                    //claw.setPosition(CLOSED_CLAW);
                })

                .addDisplacementMarker(45, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    claw.setPosition(OPEN_CLAW);
                })
                .build();

        //todo: for setposeestimate stuff change the drive.trajectorybuilder(!!!!) stuff

        // Fourth trajectory
        // Ensure that we call trajectory3.end() as the start for this one
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(new Pose2d(-57.5, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-17.5, -5.5, Math.toRadians(270)))
                .addDisplacementMarker(35, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                })
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.2);
                })
                .build();
        //wait after 4


        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d(-24, -12, Math.toRadians(270)))
                .addTemporalMarker(0.8, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.2);
                    claw.setPosition(OPEN_CLAW);
                })
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.idle();
                    claw.setPosition(OPEN_CLAW);
                })
                .addDisplacementMarker(27, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.idle();
                    arm.setPower(0.2);
                    claw.setPosition(CLOSED_CLAW);
                })

                .addDisplacementMarker(30, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    claw.setPosition(OPEN_CLAW);
                })
                .lineToLinearHeading(new Pose2d(-63, -15, Math.toRadians(177)))

                .build();

        //wait after 6

        //todo: for setposeestimate stuff change the drive.trajectorybuilder(!!!!) stuff

        TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(new Pose2d(-58, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-18.5, -14, Math.toRadians(270)))
                .addDisplacementMarker(35, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                })
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.175);
                })
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory6 = drive.trajectoryBuilder(new Pose2d(-24, -12, Math.toRadians(270)))
                .addTemporalMarker(0.8, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.2);
                    claw.setPosition(OPEN_CLAW);
                })
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.idle();
                    claw.setPosition(OPEN_CLAW);
                })
                .addDisplacementMarker(18, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.idle();
                    arm.setPower(0.45);
                    claw.setPosition(CLOSED_CLAW);
                })
                .addDisplacementMarker(25, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    claw.setPosition(OPEN_CLAW);
                })
                .lineToLinearHeading(new Pose2d(-63.5, -9, Math.toRadians(170)))

                .build();

        //todo: for setposeestimate stuff change the drive.trajectorybuilder(!!!!) stuff

        TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(-59, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-18, -12, Math.toRadians(270)))
                .addDisplacementMarker(35, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                })
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.175);
                })
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(trajectory7.end())
                .lineTo(new Vector2d(-55, -8))
                .build();

        TrajectorySequence midPark = drive.trajectorySequenceBuilder(trajectory7.end())
                .lineTo(new Vector2d(-36, -12))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-29, 2, Math.toRadians(180)))
                .lineTo(new Vector2d(-16, -8))
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
                        r = false;
                        currentState = State.TRAJECTORY_2;
                    }
                    break;
                case TRAJECTORY_2:
                    encoderStrafe(0.6, 12, -12, -12, 12, 1.0, a);
                    if (a) {
                        currentState = State.TRAJECTORY_2a;
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case TRAJECTORY_2a:
                    rotate(-getAngle(), 0.5, b);
                    waitTimer1.reset();
                    runtime.reset();
                    encoderFunction();
                    break;
                case TRAJECTORY_2b:
                    if (runtime.seconds() < 1.25) {
                        correction = checkDirection();
                        back_left.setPower(1 - correction);
                        back_right.setPower(1 + correction);
                        front_left.setPower(1 - correction);
                        front_right.setPower(1 + correction);
                        arm.setPower(-0.16);
                    }
                    if ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 30 * COUNTS_PER_INCH) < 15) &&
                            (Math.abs(Math.abs(back_right.getCurrentPosition()) - 30 * COUNTS_PER_INCH) < 15) &&
                            (Math.abs(Math.abs(front_left.getCurrentPosition()) - 30 * COUNTS_PER_INCH) < 15) &&
                            (Math.abs(Math.abs(front_right.getCurrentPosition()) - 30 * COUNTS_PER_INCH) < 15)) {
                        correction = checkDirection();
                        back_left.setPower(0.3 - correction);
                        back_right.setPower(0.3 + correction);
                        front_left.setPower(0.3 - correction);
                        front_right.setPower(0.3 + correction);
                        arm.setPower(0.6);
                    }
                    if ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 30 * COUNTS_PER_INCH) > 15) &&
                            (Math.abs(Math.abs(back_right.getCurrentPosition()) - 30 * COUNTS_PER_INCH) > 15) &&
                            (Math.abs(Math.abs(front_left.getCurrentPosition()) - 30 * COUNTS_PER_INCH) > 15) &&
                            (Math.abs(Math.abs(front_right.getCurrentPosition()) - 30 * COUNTS_PER_INCH) > 15)) {
                        currentState = State.WAIT_1a;
                        waitTimer1.reset();
                        runtime.reset();
                    }
                    break;
                case WAIT_1a:
                    if (timer.seconds() > 0.35){
                        claw.setPosition(CLOSED_CLAW);
                        arm.setPower(-0.11);
                    }
                    if (timer.seconds() > 0.5){
                        lift.setTarget(500);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= 0.75) {
                        currentState = State.TRAJECTORY_3;
                        ////lift.setTarget(300);
                        drive.setPoseEstimate(new Pose2d(-57.5, -12, Math.toRadians(180)));
                        r = true;
                        drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;

                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;

                case WAIT_2:
                    if(timer.seconds() > 0.25){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() > 0.3){
                        arm.setPower(.4);
                    }
                    if(timer.seconds() > 1){
                        lift.setTarget(100);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on
                    if (waitTimer1.seconds() >= 1.1) {
                        currentState = State.TRAJECTORY_4;
                        drive.setPoseEstimate(new Pose2d(new Vector2d(-24, -12), Math.toRadians(270)));
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;

                case TRAJECTORY_4:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_3;
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;

                case WAIT_3:

                    if (timer.seconds() > 0.1){
                        //arm.setPower(0.2);
                    }
                    if (timer.seconds() > 0.5){
                        claw.setPosition(CLOSED_CLAW);
                        arm.setPower(-0.13);
                    }

                    if (timer.seconds() > 0.65){
                        lift.setTarget(500);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= 1) {
                        currentState = State.TRAJECTORY_5;
                        drive.setPoseEstimate(new Pose2d(-58, -12, Math.toRadians(180)));
                        drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;
                case TRAJECTORY_5:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_4;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_4:
                    if(timer.seconds() > 0.2){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() > 0.3){
                        arm.setPower(.275);
                    }
                    if(timer.seconds() > 1.1){
                        lift.setTarget(100);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on
                    if (waitTimer1.seconds() >= 1.3) {
                        currentState = State.TRAJECTORY_6;
                        drive.setPoseEstimate(new Pose2d(new Vector2d(-24, -12), Math.toRadians(270)));
                        ////arm.setPower(-0.2);
                        drive.followTrajectoryAsync(trajectory6);
                    }
                    break;
                case TRAJECTORY_6:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_5;
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_5:
                    if (timer.seconds() > 0.1){
                        //arm.setPower(0.2);
                    }
                    if (timer.seconds() > 0.5){
                        claw.setPosition(CLOSED_CLAW);
                    }
                    if (timer.seconds() > 0.65){
                        arm.setPower(-0.13);
                        lift.setTarget(500);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= 1) {
                        currentState = State.TRAJECTORY_7;
                        drive.setPoseEstimate(new Pose2d(-59, -12, Math.toRadians(180)));
                        drive.followTrajectorySequenceAsync(trajectory7);
                    }
                    break;
                case TRAJECTORY_7:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    //arm.setPower(-0.2);
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_6;
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;

                case WAIT_6:
                    if(timer.seconds() > 0.2){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() > 0.3){
                        arm.setPower(.35);
                    }
                    if(timer.seconds() > 1.1){
                        lift.setTarget(100);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on
                    if (waitTimer1.seconds() >= 1.3) {
                        currentState = State.IDLE;
                        drive.setPoseEstimate(new Pose2d(new Vector2d(-24, -12), Math.toRadians(270)));
                        ////arm.setPower(-0.2);
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
            if(r) {
                drive.update();
            }
            else{

            }
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            arm.update();
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

    public void encoderFunction() {

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(75);
        telemetry.addData("Worked", 0);
    }

    public void Turnoff() {
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPZero() {
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            newLeftTarget = back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH );
            newRightTarget = back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH );
            back_left.setTargetPosition(newLeftTarget);
            front_left.setTargetPosition(newLeftTarget);
            back_right.setTargetPosition(newRightTarget);
            front_right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

        }
    }

    public void encoderStrafe(double speed,
                              double bLInches, double bRInches, double fLInches, double fRInches,
                              double timeoutS, boolean done) {
        int newBLeftTarget;
        int newBRightTarget;
        int newFLeftTarget;
        int newFRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            newBLeftTarget = back_left.getCurrentPosition() + (int)(bLInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newBRightTarget = back_right.getCurrentPosition() + (int)(bRInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newFLeftTarget = back_left.getCurrentPosition() + (int)(fLInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newFRightTarget = back_right.getCurrentPosition() + (int)(fRInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            back_left.setTargetPosition(newBLeftTarget);
            front_left.setTargetPosition(newFLeftTarget);
            back_right.setTargetPosition(newBRightTarget);
            front_right.setTargetPosition(newFRightTarget);

            // Turn On RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (back_left.isBusy() && back_right.isBusy() && front_left.isBusy() && front_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBLeftTarget,  newBRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        back_left.getCurrentPosition(), back_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            back_left.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);
            front_right.setPower(0);

            // Turn off RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(250);   // optional pause after each move.
            done = true;
        }

    }

    public void resetAngle() {
        //Intrinsic is rotation to the robot
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .105;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power, boolean done) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        Turnoff();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        back_left.setPower(leftPower);
        back_right.setPower(rightPower);
        front_left.setPower(leftPower);
        front_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);

        // wait for rotation to stop.
        sleep(150);

        // reset angle tracking on new heading.
        resetAngle();
        done = true;
    }
}