package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

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
@Autonomous(name = "1and4newmaybe", group = "advanced")
public class Auto1and4newmaybe extends LinearOpMode {

    private Servo claw;

    double OPEN_CLAW = 0.56;
    double CLOSED_CLAW = 0;
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineToSplineHeading() trajectory

        WAIT_1,         // Then we're gonna wait a second to score

        TRAJECTORY_2,   // Then, we follow a splineToConstantHeading() trajectory to move away from the pole
        TRAJECTORY_3,   // Then, we follow a splineToConstantHeading() trajectory

        WAIT_2,         // Then we're gonna wait a second to pick up a cone

        TRAJECTORY_4,   // Then we want to move backwards a bit
        TRAJECTORY_5,   // Then, we follow another splineToConstantHeading() trajectory

        WAIT_3,         // Then we're gonna wait a second to score

        TRAJECTORY_6,   // Then, we follow a splineToConstantHeading() trajectory to move away from the pole
        TRAJECTORY_7,   // Then, we follow a splineToConstantHeading() trajectory

        WAIT_4,         // Then we're gonna wait a second to pick up a cone

        TRAJECTORY_8,   // Then we want to move backwards a bit
        TRAJECTORY_9,   // Then, we follow another splineToConstantHeading() trajectory

        WAIT_5,         // Then we're gonna wait a second to score

        TRAJECTORY_10,  // Then, we follow a splineToConstantHeading() trajectory to move away from the pole
        TRAJECTORY_11,  // Then, we follow a splineToConstantHeading() trajectory

        WAIT_6,         // Then we're gonna wait a second to pick up a cone

        TRAJECTORY_12,  // Then we want to move backwards a bit
        TRAJECTORY_13,  // Then, we follow another splineToConstantHeading() trajectory

        WAIT_7,         // Then we're gonna wait a second to score

        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(90));

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
                    claw.setPosition(CLOSED_CLAW);
                })
                .splineToSplineHeading(new Pose2d(-28.5, 2, Math.toRadians(180)), Math.toRadians(45))
                .addDisplacementMarker(25, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                    arm.setPower(-.3);
                })
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .addTemporalMarker(0.25, () -> {
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
                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                .build();

        // Third trajectory
        // Ensure that we call trajectory2.end() as the start for this one
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .addDisplacementMarker(15, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.idle();
                })
                .splineToConstantHeading(new Vector2d(-63.5, -6), Math.toRadians(175))
                .addDisplacementMarker(() -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    drive.setPoseEstimate(new Pose2d(-63.5, -12, Math.toRadians(180)));
                })
                .build();

        //todo: for setposeestimate stuff change the drive.trajectorybuilder(!!!!) stuff

        // Fourth trajectory
        // Ensure that we call trajectory3.end() as the start for this one
        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d(-63.5, -12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-51, -12), Math.toRadians(0))
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(0.2);
                })
                .build();

        // Fifth trajectory
        // Ensure that we call trajectory4.end() as the start for this one
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .splineToConstantHeading(new Vector2d(-30, 1), Math.toRadians(90))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.3);
                })
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                })
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .addTemporalMarker(0.25, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.2);
                })
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.idle();
                    claw.setPosition(OPEN_CLAW);
                })
                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .addDisplacementMarker(15, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.idle();
                })
                .splineToConstantHeading(new Vector2d(-63.5, -6), Math.toRadians(175))
                .addDisplacementMarker(() -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    drive.setPoseEstimate(new Pose2d(-63.5, -12, Math.toRadians(180)));
                })
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(new Pose2d(-63.5, -12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-51, -12), Math.toRadians(0))
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(0.2);
                })
                .build();

        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .splineToConstantHeading(new Vector2d(-30, 1), Math.toRadians(90))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.3);
                })
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                })
                .build();

        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .addTemporalMarker(0.25, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.2);
                })
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.idle();
                    claw.setPosition(OPEN_CLAW);
                })
                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(180))
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(trajectory10.end())
                .addDisplacementMarker(15, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.idle();
                })
                .splineToConstantHeading(new Vector2d(-63.5, -6), Math.toRadians(175))
                .addDisplacementMarker(() -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    drive.setPoseEstimate(new Pose2d(-63.5, -12, Math.toRadians(180)));
                })
                .build();

        Trajectory trajectory12 = drive.trajectoryBuilder(new Pose2d(-63.5, -12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-51, -12), Math.toRadians(0))
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                    arm.setPower(0.2);
                })
                .build();

        Trajectory trajectory13 = drive.trajectoryBuilder(trajectory12.end())
                .splineToConstantHeading(new Vector2d(-30, 1), Math.toRadians(90))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    arm.setPower(-0.3);
                })
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    // Run your action in here!
                    lift.setTarget(2400);
                })
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime waitTimer1 = new ElapsedTime();

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
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_1:
                    if(timer.seconds() > 0.2){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() > 0.3){
                        arm.setPower(.35);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on
                    if (waitTimer1.seconds() >= 1) {
                        currentState = State.TRAJECTORY_2;
                        lift.setTarget(100);
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_2:
                    if (timer.seconds() > 0.1){
                        claw.setPosition(CLOSED_CLAW);
                    }
                    if (timer.seconds() > 1){
                        arm.setPower(-0.2);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;
                case TRAJECTORY_4:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_5;
                        drive.followTrajectoryAsync(trajectory5);
                    }
                    break;
                case TRAJECTORY_5:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_3;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_3:
                    if(timer.seconds() < 0.2){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() < 1){
                        arm.setPower(1);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectoryAsync(trajectory6);
                    }
                    break;





                case TRAJECTORY_6:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7;
                        drive.followTrajectoryAsync(trajectory7);
                    }
                    break;
                case TRAJECTORY_7:
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
                    if (timer.seconds() > 0.1){
                        claw.setPosition(CLOSED_CLAW);
                    }
                    if (timer.seconds() > 1){
                        arm.setPower(-0.2);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_8;
                        drive.followTrajectoryAsync(trajectory8);
                    }
                    break;
                case TRAJECTORY_8:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_9;
                        drive.followTrajectoryAsync(trajectory9);
                    }
                    break;
                case TRAJECTORY_9:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_5;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_5:
                    if(timer.seconds() < 0.2){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() < 1){
                        arm.setPower(1);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_10;
                        drive.followTrajectoryAsync(trajectory10);
                    }
                    break;
                case TRAJECTORY_10:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_11;
                        drive.followTrajectoryAsync(trajectory11);
                    }
                    break;
                case TRAJECTORY_11:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_6;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_6:
                    if (timer.seconds() > 0.1){
                        claw.setPosition(CLOSED_CLAW);
                    }
                    if (timer.seconds() > 1){
                        arm.setPower(-0.2);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_12;
                        drive.followTrajectoryAsync(trajectory12);
                    }
                    break;
                case TRAJECTORY_12:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_13;
                        drive.followTrajectoryAsync(trajectory13);
                    }
                    break;
                case TRAJECTORY_13:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_7;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        timer.reset();
                    }
                    break;
                case WAIT_7:
                    if(timer.seconds() < 0.2){
                        claw.setPosition(OPEN_CLAW);
                    }
                    if(timer.seconds() < 1){
                        arm.setPower(1);
                    }
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.IDLE;
                        lift.setTarget(200);
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
}