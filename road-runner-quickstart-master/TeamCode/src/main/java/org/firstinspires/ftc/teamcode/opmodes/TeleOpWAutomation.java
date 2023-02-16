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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Lift;

@TeleOp(name = "automated stuff")
public class TeleOpWAutomation extends LinearOpMode {

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
    boolean update = false;
    ElapsedTime runtime = new ElapsedTime();
    public enum State{
        START,
        EXTEND,
        WAITPRESS,
        WAITMOVE,
        RETRACT
    }
    State state = State.START;
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
        Lift lift = new Lift(hardwareMap);
        // Put initialization blocks here.
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);
        arm_right.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //clawopen checks if it's being used to open or close
        //true = open
        //false = close
        boolean clawopen = false;
        claw.setPosition(0);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                double slides = -gamepad2.right_stick_y;
                switch(state){
                    case START:
                        if (slides > 0.2) {
                            lift.setLiftPowers(0.75);
                            lift.setLiftPowers(0.75);
                        } else if (slides < -0.2) {
                            lift.setLiftPowers(-0.75);
                            lift.setLiftPowers(-0.75);
                        } else {
                            lift.setLiftPowers(0.04);
                            lift.setLiftPowers(0.04);
                        }
                        if (gamepad2.left_trigger > 0.2 || gamepad2.x) {
                            claw.setPosition(.48);
                            clawopen = true;

                        } else if (gamepad2.right_trigger > 0.2 || gamepad2.b) {
                            claw.setPosition(0);
                            clawopen = false;
                        }
                        update = false;
                        if(gamepad2.y){
                            lift.setTarget(2350);
                            update = true;
                            state = State.EXTEND;
                        }
                        break;
                    case EXTEND:
                        telemetry.addData("slideL ", Math.abs(slide_left.getCurrentPosition()));
                        telemetry.addData("target ", 2350);
                        telemetry.addData("whole ", Math.abs(Math.abs(slide_left.getCurrentPosition()) - 2350));
                        telemetry.addData("true? ", (Math.abs(Math.abs(slide_left.getCurrentPosition()) - 2350) < 10));
                        if(Math.abs(Math.abs(slide_left.getCurrentPosition()) - 2350) < 10){
                            update = false;
                            state = State.WAITPRESS;
                        }
                        break;
                    case WAITPRESS:
                        if (slides > 0.2) {
                            lift.setLiftPowers(0.75);
                            lift.setLiftPowers(0.75);
                        } else if (slides < -0.2) {
                            lift.setLiftPowers(-0.75);
                            lift.setLiftPowers(-0.75);
                        } else {
                            lift.setLiftPowers(0.04);
                            lift.setLiftPowers(0.04);
                        }
                        if(gamepad2.left_trigger > 0.2 || gamepad2.x){
                            claw.setPosition(0.48);
                            clawopen = true;
                            runtime.reset();
                            state = State.WAITMOVE;
                        }
                        break;
                    case WAITMOVE:
                        if(runtime.seconds() > 3){
                            lift.setTarget(0);
                            update = true;
                            state = State.RETRACT;
                        }
                        if (gamepad2.left_trigger > 0.2 || gamepad2.x) {
                            claw.setPosition(.48);
                            clawopen = true;

                        } else if (gamepad2.right_trigger > 0.2 || gamepad2.b) {
                            claw.setPosition(0);
                            clawopen = false;
                        }
                        break;
                    case RETRACT:
                        if(Math.abs(Math.abs(back_left.getCurrentPosition()) - 0) < 10){
                            update = false;
                            lift.stopReset();
                            state = State.START;
                        }
                        break;
                    default:
                        state = State.START;
                }
                if((gamepad2.left_bumper || gamepad1.left_bumper || gamepad2.dpad_down) && state != State.START){
                    state = State.START;
                }
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.right_stick_x;
                double rotate = 0.65 * gamepad1.left_stick_x;
                double arms = -gamepad2.left_stick_y;

                double bLPower = forward - strafe + rotate; //
                double bRPower = forward + strafe - rotate; //
                double fLPower = forward + strafe + rotate; //
                double fRPower = forward - strafe - rotate; // - strafe
                setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                // reset speed variables

                if (arms > 0.2) {
                    arm_left.setPower(1);
                    arm_right.setPower(1);
                } else if (arms < -0.2) {
                    arm_left.setPower(-1);
                    arm_right.setPower(-1);
                } else {
                    arm_left.setPower(0.0);
                    arm_right.setPower(0.0);
                }
                if(update){
                    lift.update();
                }
                if((gamepad2.right_bumper && gamepad2.left_bumper) || (gamepad1.right_bumper && gamepad1.left_bumper)){
                    setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                    arm_left.setPower(0.0);
                    arm_right.setPower(0.0);
                    slide_left.setPower(0.04);
                    slide_right.setPower(0.04);
                    sleep(500);
                }
                // set motor parameters to driver station
                telemetry.addData("left slide pos: ", slide_left.getCurrentPosition());
                telemetry.addData("right slide pos: ", slide_right.getCurrentPosition());
                telemetry.addData("Clawpower: ", claw.getPosition());
                telemetry.addData("ClawOpen: ",clawopen);
                telemetry.addData("forward: ", forward);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);
                telemetry.addData("state ", state);
                telemetry.update();
            }
        }
        telemetry.update();
    }

}
