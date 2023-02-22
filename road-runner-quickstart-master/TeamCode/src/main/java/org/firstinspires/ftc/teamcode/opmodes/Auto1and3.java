package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
@Autonomous(name = "AsyncFollowingFSM", group = "advanced")
@Disabled
public class AsyncFollowingFSM extends LinearOpMode {
    private Servo claw;

    final double OPEN_CLAW = .46;
    final double CLOSED_CLAW = 0;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   //drive to pole
        WAIT_1, //turn to score
        TRAJECTORY_2, //drive to stack
        TRAJECTORY_3, //drive to pole
        TRAJECTORY_4, //drive to stack
        TRAJECTORY_5, //drive to pole
        TRAJECTORY_6, //drive to stack
        TRAJECTORY_7, //drive to pole
        TRAJECTORY_8, //drive to stack
        TRAJECTORY_9, //park mid
        TRAJECTORY_10, //park right
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(0));

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
                .lineTo(new Vector2d(-34, 0))
                .build();

        double turnAngle1 = Math.toRadians(90);
        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, turnAngle1)))
                .strafeLeft(4)
                .splineToConstantHeading(new Vector2d(-63, -12), Math.toRadians(180))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .splineToSplineHeading(new Pose2d(-63, -13, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-27, -5, Math.toRadians(225)), Math.toRadians(45))
                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .splineToSplineHeading(new Pose2d(-63, -14, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(200)), Math.toRadians(45))
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .splineToSplineHeading(new Pose2d(-63, -16, Math.toRadians(180)), Math.toRadians(180))
                .build();


        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();
        ElapsedTime downArmTimer = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);

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
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TURN_1:
                    lift.setTarget(2400);
                    arm.setPower(-1);
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        downArmTimer.reset();
                        currentState = State.WAIT_1;
                    }
                    break;
                case WAIT_1:
                    if(downArmTimer.seconds() >0.5){
                        claw.setPosition(OPEN_CLAW);
                        downArmTimer.reset();
                    }
                    if(downArmTimer.seconds() >0.5){
                        claw.setPosition(CLOSED_CLAW);
                        arm.setPower(1);
                        drive.followTrajectoryAsync(trajectory2);
                        currentState = State.TRAJECTORY_2;
                    }
                case TRAJECTORY_2:
                    lift.setTarget(900);
                    downArmTimer.reset();
                    if(downArmTimer.seconds() >0.5){
                        claw.setPosition(OPEN_CLAW);
                        downArmTimer.reset();
                    }
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    claw.setPosition(CLOSED_CLAW);
                    arm.setPower(-1);
                    lift.setTarget(2400);
                    downArmTimer.reset();
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;
                case TRAJECTORY_4:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        downArmTimer.reset();
                    }
                    break;
                case WAIT_1:
                    arm_left.setPower(-0.75);
                    arm_right.setPower(-0.75);
                    if(downArmTimer.seconds() >= 1){
                        claw.setPosition(CLOSED_CLAW);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_5;
                        drive.followTrajectoryAsync(trajectory5);
                    }
                    break;
                case TRAJECTORY_5:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectoryAsync(trajectory6);
                    }
                    break;
                case TRAJECTORY_6:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7;
                        drive.followTrajectoryAsync(trajectory7);
                    }
                    break;
                case TRAJECTORY_7:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_8;
                        drive.followTrajectoryAsync(trajectory8);
                    }
                    break;
                case TRAJECTORY_8:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        downArmTimer.reset();
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
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
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
}