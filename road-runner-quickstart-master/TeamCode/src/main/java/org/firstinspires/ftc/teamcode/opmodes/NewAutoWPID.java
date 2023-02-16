package org.firstinspires.ftc.teamcode.opmodes;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.DriveWAllPID;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name = "NewAutoWPID")
@Disabled
public class NewAutoWPID extends LinearOpMode {
    //Time
    private ElapsedTime runtime = new ElapsedTime();

    //Wheels
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private Servo claw;
    double target, _correction;
    private IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .5, correction;
    private ElapsedTime clawtime = new ElapsedTime();

    private CRServo arm_left;
    private CRServo arm_right;
    //Cameras
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    private DcMotorEx slide_left;
    private DcMotorEx slide_right;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    
    // UNITS ARE METERS
    double tagsize = 0.166;

    //6:1
    //16:2
    //12:3
    int LeftTag = 6; // Tag ID 18 from the 36h11 family
    int RightTag = 12;
    int MiddleTag = 16;
    int location = 0;

    AprilTagDetection tagOfInterest = null;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        //Gets Hardware
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        arm_left = hardwareMap.get(CRServo.class, "arm_left");
        arm_right = hardwareMap.get(CRServo.class, "arm_right");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //Constant Variables
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_left = hardwareMap.get(DcMotorEx.class, "slide_left");
        slide_right = hardwareMap.get(DcMotorEx.class, "slide_right");
        arm_right.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift lift = new Lift(hardwareMap);
        Drive wheels = new Drive(hardwareMap);
        //SetWheelsToZero
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheels.encoderFunction();
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        //PlayButton

        claw.setPosition(0);
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            runtime.reset();
            resetAngle();
            wheels.setTarget(40.25);
            while (!(Math.abs(Math.abs(back_left.getCurrentPosition()) - 40.25 * COUNTS_PER_INCH * 30.0 / 30.75) < 10) && opModeIsActive()) {
                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                correction = checkDirection();
                wheels.move(correction);
                telemetry.addData("bl pos ", wheels.getBackLPos());
                telemetry.addData("target ", wheels.getTarget());
                telemetry.addData("error ", wheels.getTarget() - wheels.getBackLPos());
                telemetry.update();
            }
            wheels.brake();
            //todo: see if rotate/encoderStrafe/drivetrain works outside of Drive class
            rotate(-90, 0.3);
            lift.setTarget(2150);
            runtime.reset();
            while (runtime.seconds() < 1.5 && opModeIsActive()) {
                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                lift.update();
            }
            wheels.encoderFunction();
            runtime.reset();
            wheels.setTarget(5);
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 5 * COUNTS_PER_INCH * 30.0 / 30.75) > 10) && opModeIsActive()) {

                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                lift.update();
                correction = checkDirection();
                wheels.move(correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 5 * COUNTS_PER_INCH * 30.0 / 30.75);
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 5 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            wheels.brake();
            wheels.encoderFunction();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 0.5) {

                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                lift.update();
                claw.setPosition(0.48);

            }
            runtime.reset();
            wheels.setTarget(-5);
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 5 * COUNTS_PER_INCH * 30.0 / 30.75) > 10) && opModeIsActive()) {

                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                lift.update();
                correction = checkDirection();
                wheels.move(correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 6 * COUNTS_PER_INCH * 30.0 / 30.75);
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            wheels.brake();
            wheels.encoderFunction();
            lift.setTarget(0);
            runtime.reset();
            while (runtime.seconds() < 2 && opModeIsActive()) {
                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                lift.update();
            }
            rotate(90, 0.3);
            wheels.encoderFunction();
            wheels.setTarget(-14);
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 14 * COUNTS_PER_INCH * 30.0 / 30.75) > 10) && opModeIsActive()){

                arm_left.setPower(0.02);
                arm_right.setPower(0.02);
                lift.update();
                correction = checkDirection();
                wheels.move(correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 6* COUNTS_PER_INCH * 30.0 / 30.75);
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            if(location==1){
                encoderStrafe(0.5, 26, -26, -26, 26, 3.0);
            }
            else if(location == 3){
                encoderStrafe(0.5, -24, 24, 24, -24, 3.0);
            }

            /*
            encoderDrive(.5, 3, 3, 1.5);
            rotate(-getAngle(), 0.3);
            rotate(90, 0.3);
            encoderDrive(DRIVE_SPEED, 35, 35, 4.0);
            rotate(-getAngle(), 0.3);
            claw.setPosition(0.5);
            clawtime.reset();
            if (clawtime.seconds() <= 1.0){

            }
            encoderDrive(DRIVE_SPEED, -35, -35, 4.0);
            rotate(-getAngle(), 0.3);
            rotate(-90, 0.3);
            */
            //Left
            /*
            if (location == 1) {
                //sleep(20000);
                encoderDrive(.5,28,28 ,5);
                rotate(-getAngle(), 0.3);
                encoderStrafe(0.6, 24, -24, -24, 24, 4.0);
                rotate(-getAngle(), 0.3);
                encoderDrive(DRIVE_SPEED, 4, 4, 2.0);
                requestOpModeStop();
            } else if (location == 2) {
                encoderDrive(.5, 28, 28, 10);
                rotate(-getAngle(), 0.3);
                requestOpModeStop();
            } else if (location == 3) {
                //sleep(20000);
                encoderStrafe(.6,-22,22, 22, -22, 3.0);
                rotate(-getAngle(), 0.3);
                encoderDrive(.5, 26, 26, 5.0);
                rotate(-getAngle(), 0.3);
                requestOpModeStop();
            }
            location = 0;

             */


        }
    }
    /* Update the telemetry */


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

    public void Turnoff() {
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderStrafe(double speed,
                              double bLInches, double bRInches, double fLInches, double fRInches,
                              double timeoutS) {
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
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
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
        double correction, angle, gain = .10;

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
    public void rotate(double degrees, double power) {
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
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}