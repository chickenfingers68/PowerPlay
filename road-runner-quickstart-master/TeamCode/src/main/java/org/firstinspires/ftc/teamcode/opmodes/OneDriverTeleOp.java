package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OneDriver")
public class OneDriverTeleOp extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private Servo claw;
    private CRServo arm_left;
    private CRServo arm_right;
    private DcMotor slide_left;
    private DcMotor slide_right;
    private IMU imu;


    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private void setDrivePowers(double bLPower, double bRPower, double fLPower, double fRPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));

        bLPower /= maxSpeed;
        bRPower /= maxSpeed;
        fLPower /= maxSpeed;
        fRPower /= maxSpeed;

        back_left.setPower(bLPower);
        back_right.setPower(bRPower);
        front_left.setPower(fLPower);
        front_right.setPower(fRPower);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        claw = hardwareMap.get(Servo.class, "claw");
        arm_left = hardwareMap.get(CRServo.class, "arm_left");
        arm_right = hardwareMap.get(CRServo.class, "arm_right");
        slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        slide_right = hardwareMap.get(DcMotor.class, "slide_right");

        // Put initialization blocks here.
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);
        arm_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //clawopen checks if it's being used to open or close
        //true = open
        //false = close
        boolean clawopen = false;
        claw.setPosition(1);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                double forward;
                double strafe;
                if(-1 * gamepad2.left_stick_y > 0.2 || -1 * gamepad2.left_stick_y < -0.2){
                    forward = -0.7 * gamepad2.left_stick_y;
                }
                else{
                    forward = 0;
                }
                if(gamepad2.left_stick_x > 0.2 || gamepad2.left_stick_x < -0.2){
                    strafe = 0.7 * gamepad2.left_stick_x;
                }
                else{
                    strafe = 0;
                }
                double rotate = 0.55 * (gamepad2.right_trigger - gamepad2.left_trigger);
                double slides = -gamepad2.right_stick_y;
                double arms = -gamepad2.left_stick_y;

                double bLPower = forward - strafe + rotate; //
                double bRPower = forward + strafe - rotate; //
                double fLPower = forward + strafe + rotate; //
                double fRPower = forward - strafe - rotate; // - strafe
                setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                // reset speed variables
                if (gamepad2.x) {
                    claw.setPosition(0.82);
                    clawopen = true;

                } else if (gamepad2.b) {
                    claw.setPosition(1);
                    clawopen = false;
                }

                if (gamepad2.dpad_up) {
                    arm_left.setPower(1);
                    arm_right.setPower(1);
                } else if (gamepad2.dpad_down) {
                    arm_left.setPower(-1);
                    arm_right.setPower(-1);
                } else {
                    arm_left.setPower(0.02);
                    arm_right.setPower(0.02);
                }
                if (slides > 0.2) {
                    slide_left.setPower(0.75);
                    slide_right.setPower(0.75);
                } else if (slides < -0.2) {
                    slide_left.setPower(-0.75);
                    slide_right.setPower(-0.75);
                } else {
                    slide_left.setPower(0.04);
                    slide_right.setPower(0.04);
                }
                // set motor parameters to driver station
                telemetry.addData("left slide pos: ", slide_left.getCurrentPosition());
                telemetry.addData("right slide pos: ", slide_right.getCurrentPosition());
                telemetry.addData("Clawpower: ", claw.getPosition());
                telemetry.addData("ClawOpen: ",clawopen);
                telemetry.addData("forward: ", forward);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);

                telemetry.addData("distance ", back_left.getCurrentPosition() / (COUNTS_PER_INCH * 30 / 30.75));
                telemetry.update();
                telemetry.update();
            }
        }
        telemetry.update();
    }

}
