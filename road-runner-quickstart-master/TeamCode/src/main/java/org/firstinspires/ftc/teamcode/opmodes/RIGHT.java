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

@Autonomous(name = "RIGHTold", preselectTeleOp = "ActualTeleOp")
@Disabled
public class RIGHT extends LinearOpMode {
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
    double tagsize = 0.266;

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
        //SetWheelsToZero
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderFunction();
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
        waitForStart(); //1100, 420, 900 for slides

        if (opModeIsActive() && !isStopRequested()) {
            resetAngle();
            resetAngle();
            encoderFunction();
            encoderStrafe(0.6, 5.25, -5.25, -5.25, 5.25, 1.5);
            setPZero();
            //encoderFunction();
            //rotate(-getAngle(), 0.5);
            encoderFunction();
            runtime.reset();
            while(runtime.seconds() < 0.3  && opModeIsActive()){
                arm_left.setPower(1);
                arm_right.setPower(1);
            }
            //move towards pole
            runtime.reset();
            while(runtime.seconds() < 1.75  && opModeIsActive()){
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                correction = checkDirection();
                back_left.setPower(0.45 - correction);
                back_right.setPower(0.45 + correction);
                front_left.setPower(0.45 - correction);
                front_right.setPower(0.45 + correction);
                telemetry.addData("distance ", back_left.getCurrentPosition() / (COUNTS_PER_INCH * 30 / 30.75));
                telemetry.update();
            }
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 43. * COUNTS_PER_INCH) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 43. * COUNTS_PER_INCH) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 43. * COUNTS_PER_INCH) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 43. * COUNTS_PER_INCH) > 15) &&
                    opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                correction = checkDirection();
                back_left.setPower(0.3 - correction);
                back_right.setPower(0.3 + correction);
                front_left.setPower(0.3 - correction);
                front_right.setPower(0.3 + correction);
                telemetry.addData("distance ", back_left.getCurrentPosition() / (COUNTS_PER_INCH * 30 / 30.75));
                telemetry.update();
            }
            setPZero();
            encoderFunction();
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 2 * COUNTS_PER_INCH) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 2 * COUNTS_PER_INCH) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 2 * COUNTS_PER_INCH) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 2 * COUNTS_PER_INCH) > 15) &&
                    opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                correction = checkDirection();
                back_left.setPower(-0.3 - correction);
                back_right.setPower(-0.3 + correction);
                front_left.setPower(-0.3 - correction);
                front_right.setPower(-0.3 + correction);
                telemetry.addData("distance ", back_left.getCurrentPosition() / (COUNTS_PER_INCH * 30 / 30.75));
                telemetry.update();
            }
            setPZero();
            rotate(-getAngle(), 0.5);
            encoderFunction();
            //turn to pole
            rotate(90, 0.45);
            rotate(-3.5, 0.2);
            lift.setTarget(2150);
            runtime.reset();
            //lift slides
            while (runtime.seconds() < 1.5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            encoderFunction();
            runtime.reset();
            //move fwd to pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6. * COUNTS_PER_INCH ) > 7) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6. * COUNTS_PER_INCH ) > 7) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6. * COUNTS_PER_INCH ) > 7) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6.  * COUNTS_PER_INCH ) > 7) &&
                    opModeIsActive() && runtime.seconds() < 2) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.2 - correction);
                back_right.setPower(0.2 + correction);
                front_left.setPower(0.2 - correction);
                front_right.setPower(0.2 + correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 5 * COUNTS_PER_INCH );
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 5 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            encoderFunction();
            setPZero();
            runtime.reset();
            //drop cone
            claw.setPosition(0.48);
            while (opModeIsActive() && runtime.seconds() < 0.75) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();

            }
            encoderFunction();
            runtime.reset();
            //move away from pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6.05 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6.05 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6.05 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6.05 * COUNTS_PER_INCH ) > 9) &&
                    opModeIsActive()) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.3 - correction);
                back_right.setPower(-0.3 + correction);
                front_left.setPower(-0.3 - correction);
                front_right.setPower(-0.3 + correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 6 * COUNTS_PER_INCH );
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            setPZero();
            lift.setTarget(0);
            runtime.reset();
            //retract lift
            while (runtime.seconds() < 0.3 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            runtime.reset();
            while (runtime.seconds() < 1 && opModeIsActive()) {
                arm_left.setPower(0);
                arm_right.setPower(0);
                lift.update();
            }
            resetAngle();
            //rotate to face other alliance
            rotate(-90, 0.45);
            encoderFunction();
            //move forward and knowck signal sleeve out of way
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 14.75 * COUNTS_PER_INCH ) > 10) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 14.75 * COUNTS_PER_INCH ) > 10) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 14.75 * COUNTS_PER_INCH ) > 10) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 14.75 * COUNTS_PER_INCH ) > 10) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.3 - correction);
                back_right.setPower(0.3 + correction);
                front_left.setPower(0.3 - correction);
                front_right.setPower(0.3 + correction);
                claw.setPosition(0);
                telemetry.update();
            }
            setPZero();
            rotate(-getAngle(), 0.3);
            encoderFunction();
            //move back a bit
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 1.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 1.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 1.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 1.75 * COUNTS_PER_INCH ) > 9) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.3 - correction);
                back_right.setPower(-0.3 + correction);
                front_left.setPower(-0.3 - correction);
                front_right.setPower(-0.3 + correction);
                claw.setPosition(0.48);
                telemetry.update();
            }
            setPZero();
            //turn to face cone stack
            rotate(-90, 0.45);
            //fix angle
            rotate(2, 0.2);
            encoderFunction();
            runtime.reset();
            //start moving twd cone stack
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 19.75 * COUNTS_PER_INCH ) > 20) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 19.75 * COUNTS_PER_INCH ) > 20) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 19.75 * COUNTS_PER_INCH ) > 20) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 19.75 * COUNTS_PER_INCH ) > 20) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.6 - correction);
                back_right.setPower(0.6 + correction);
                front_left.setPower(0.6 - correction);
                front_right.setPower(0.6 + correction);
                telemetry.update();
            }
            setPZero();
            lift.setTarget(1100);
            runtime.reset();
            //lift lift
            while (runtime.seconds() < 0.3 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            setPZero();
            encoderFunction();
            encoderFunction();
            runtime.reset();
            //finish move to stack
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 12.25 * COUNTS_PER_INCH ) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 12.25 * COUNTS_PER_INCH ) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 12.25 * COUNTS_PER_INCH ) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 12.25 * COUNTS_PER_INCH ) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.5 - correction);
                back_right.setPower(0.5 + correction);
                front_left.setPower(0.5 - correction);
                front_right.setPower(0.5 + correction);
                telemetry.update();
            }
            setPZero();
            //drop lift
            lift.setTarget(410);
            runtime.reset();
            while (runtime.seconds() < .4 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            //grab cone
            runtime.reset();
            claw.setPosition(0);
            while (runtime.seconds() < .4 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            //lift lift
            lift.setTarget(900);
            runtime.reset();
            while (runtime.seconds() < 0.6 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            encoderFunction();
            runtime.reset();
            //move back toward pole
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 16.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 16.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 16.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 16.75 * COUNTS_PER_INCH ) > 9) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.3 - correction);
                back_right.setPower(-0.3 + correction);
                front_left.setPower(-0.3 - correction);
                front_right.setPower(-0.3 + correction);
                telemetry.update();
            }





            setPZero();
            encoderFunction();
            //rotate toward alliance
            rotate(-90, 0.45);
            //rotate(.5, 0.3);
            lift.setTarget(1300);
            runtime.reset();
            //lift slides
            while (runtime.seconds() < 0.75 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            rotate(1, 0.2);
            encoderFunction();
            //drive forward to pole
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 7.1 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 7.1 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 7.1 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 7.1 * COUNTS_PER_INCH ) > 9) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.3 - correction);
                back_right.setPower(0.3 + correction);
                front_left.setPower(0.3 - correction);
                front_right.setPower(0.3 + correction);
                telemetry.update();
            }
            //rotate(3, 0.4);
            setPZero();
            encoderFunction();
            lift.setTarget(900);
            runtime.reset();
            while (runtime.seconds() < .5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            runtime.reset();
            //drop
            claw.setPosition(0.48);
            while (opModeIsActive() && runtime.seconds() < 0.25) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();

            }
            lift.setTarget(1100);
            runtime.reset();
            while (runtime.seconds() < .35 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            encoderFunction();
            runtime.reset();
            //move away from pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6.75 * COUNTS_PER_INCH ) > 9) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6.75 * COUNTS_PER_INCH ) > 9) &&
                    opModeIsActive()) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.3 - correction);
                back_right.setPower(-0.3 + correction);
                front_left.setPower(-0.3 - correction);
                front_right.setPower(-0.3 + correction);
            }
            setPZero();
            lift.setTarget(0);
            runtime.reset();
            //retract slides
            while (runtime.seconds() < 0.3 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            lift.idle();
            encoderFunction();
            if(location==1){
                lift.idle();
                setPZero();
                encoderFunction();
                encoderStrafe(1, -35, 35, 35, -35, 2.0);
                setPZero();
                encoderFunction();
                //move back a bit
                while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH ) > 20) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6 * COUNTS_PER_INCH ) > 20) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH ) > 20) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6 * COUNTS_PER_INCH ) > 20) &&
                        opModeIsActive()){

                    arm_left.setPower(0.03);
                    arm_right.setPower(0.03);
                    lift.update();
                    correction = checkDirection();
                    back_left.setPower(1 - correction);
                    back_right.setPower(1 + correction);
                    front_left.setPower(1 - correction);
                    front_right.setPower(1 + correction);
                    telemetry.update();
                }
                encoderStrafe(1, 3, -3, -3, 3, 2.0);
            }
            else if(location == 2){

                lift.idle();
                setPZero();
                encoderFunction();
                encoderStrafe(1, -12, 12, 12, -12, 2.0);
                setPZero();
            }
            else if (location == 3) {

                lift.idle();
                setPZero();
                encoderFunction();
                encoderStrafe(1, 12, -12, -12, 12, 2.0);
                setPZero();
            }
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

            sleep(150);   // optional pause after each move.
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
        sleep(150);

        // reset angle tracking on new heading.
        resetAngle();
    }
}