package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SlideTests")
@Disabled
public class ActualTeleOpMaybe extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private Servo claw;
    private CRServo arm_left;
    private CRServo arm_right;
    private DcMotor slide_left;
    private DcMotor slide_right;

    double motorMax;

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

        //clawopen checks if it's being used to open or close
        //true = open
        //false = close
        boolean clawopen = true;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.right_stick_x;
                double rotate = 0.65 * gamepad1.left_stick_x;
                double slides = -gamepad2.right_stick_y;
                double arms = -gamepad2.left_stick_y;

                double bLPower = forward - strafe + rotate; //
                double bRPower = forward + strafe - rotate; //
                double fLPower = forward + strafe + rotate; //
                double fRPower = forward - strafe - rotate; // - strafe
                setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                // reset speed variables
                if (gamepad2.left_trigger > 0.2 || gamepad2.x) {
                    claw.setPosition(.48);
                    clawopen = false;

                } else if (gamepad2.right_trigger > 0.2 || gamepad2.b) {
                    claw.setPosition(0);
                    clawopen = true;
                }

                if (-gamepad2.left_stick_y > 0.2) {
                    arm_left.setPower(1);
                    arm_right.setPower(-1);
                } else if (-gamepad2.left_stick_y < -0.2) {
                    arm_left.setPower(-1);
                    arm_right.setPower(1);
                } else {
                    arm_left.setPower(0.0);
                    arm_right.setPower(-0.0);
                }
                if (gamepad2.right_stick_y < -0.2) {
                    slide_left.setPower(-0.65);
                    slide_right.setPower(-0.65);
                } else if (gamepad2.right_stick_y > 0.2) {
                    slide_left.setPower(0.65);
                    slide_right.setPower(0.65);
                } else {
                    slide_left.setPower(-0.04);
                    slide_right.setPower(-0.04);
                }
                // set motor parameters to driver station
                telemetry.addData("left slide pos: ", slide_left.getCurrentPosition());
                telemetry.addData("right slide pos: ", slide_right.getCurrentPosition());
                telemetry.addData("Clawpower: ", claw.getPosition());
                telemetry.addData("ClawOpen: ",clawopen);
                telemetry.addData("forward: ", forward);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);
                telemetry.update();
            }
        }
        telemetry.update();
    }

}
