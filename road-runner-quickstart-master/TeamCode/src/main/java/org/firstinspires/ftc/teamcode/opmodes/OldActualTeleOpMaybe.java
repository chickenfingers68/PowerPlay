package org.firstinspires.ftc.teamcode.opmodes;

// import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.WholeRobot;

@TeleOp(name="OldActualTeleMaybe", group="1a")
@Disabled
public class OldActualTeleOpMaybe extends OpMode {

    public enum LiftTurretState {
        LIFT_START,
        LIFT_LOW,
        LIFT_MID,
        LIFT_HIGH,
        LIFT_RETRACT,
        LIFT_FLIPBACK,
        LIFT_FLIPJUNCTION,
        LIFT_FLIPMID
    };

    LiftTurretState liftTurretState = LiftTurretState.LIFT_START;
    ElapsedTime v4bTimer = new ElapsedTime();
    //time for v4b to finish flipping
    final double FLIP_TIME = 1.05;
    final int BOTTOMPOS = 0; //make sure slides at bottom at end of auto
    //ticks to reach low position
    final int LOWPOS = 400;
    //ticks to reach mid pos
    final int MIDPOS = 1200;
    //ticks to reach high pos
    final int HIGHPOS = 2400;
    final double OPEN_CLAW = .73; //todo
    final double CLOSED_CLAW = 0; //todo

    private PIDController controller;
    public boolean clawOpen = false;

    public static double p = 0.01, i = 0, d = 0; //tuned in other opmode
    public static double f = 0;

    public static int target = 0;

    private DcMotorEx slide_left;
    private DcMotorEx slide_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private CRServo arm_left;
    private CRServo arm_right;
    private Servo claw;

    public void Turnoff()
    {
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderSet()
    {
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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

    @Override
    public void init() {
        v4bTimer.reset();
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slide_left = hardwareMap.get(DcMotorEx.class, "slide_left");
        slide_right = hardwareMap.get(DcMotorEx.class, "slide_right");
        slide_left.setDirection(DcMotor.Direction.REVERSE);
        slide_right.setDirection(DcMotor.Direction.REVERSE);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_left = hardwareMap.get(CRServo.class, "arm_left");
        arm_right = hardwareMap.get(CRServo.class, "arm_right");
        arm_left.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLOSED_CLAW);
    }

    @Override
    public void loop() {
        /*switch(liftTurretState) {
            case LIFT_START:
                back_left.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                front_right.setPower(0);
                telemetry.addData("state", "start");
                Turnoff();
                //waiting for input
                //a ground, x low, b mid, y high
                if(gamepad2.a){ //if a pressed, start flipping
                    //code to flip v4b to ground junction
                    arm_left.setPower(-0.6);
                    arm_right.setPower(-0.6);
                    telemetry.addData("a pressed ", "ground");
                    v4bTimer.reset();
                    liftTurretState = LiftTurretState.LIFT_FLIPJUNCTION;
                }
                else if(gamepad2.x){
                    //code to extend to low pos
                    slide_left.setTargetPosition(-LOWPOS);
                    slide_right.setTargetPosition(LOWPOS);
                    slide_left.setPower(0.2);
                    slide_right.setPower(0.2);
                    telemetry.addData("x pressed ", "low");
                    liftTurretState = LiftTurretState.LIFT_LOW;
                }
                else if(gamepad2.b){
                    //code to extend to mid pos
                    encoderSet();
                    slide_left.setTargetPosition(-MIDPOS);
                    slide_right.setTargetPosition(MIDPOS);
                    slide_left.setPower(0.2);
                    slide_right.setPower(0.2);
                    telemetry.addData("b pressed ", "mid");
                    liftTurretState = LiftTurretState.LIFT_MID;
                }
                else if (gamepad2.y){
                    //code to extend to high pos
                    encoderSet();
                    slide_left.setTargetPosition(-HIGHPOS);
                    slide_right.setTargetPosition(HIGHPOS);
                    slide_left.setPower(0.2);
                    slide_right.setPower(0.2);
                    telemetry.addData("x pressed ", "high");
                    liftTurretState = LiftTurretState.LIFT_HIGH;
                }
                else if (gamepad2.dpad_down){
                    arm_left.setPower(-0.6);
                    arm_right.setPower(-0.6);
                    v4bTimer.reset();
                    liftTurretState = LiftTurretState.LIFT_FLIPMID;
                }
                break;
            case LIFT_LOW:
                //wait for lift to finish extending
                telemetry.addData("state: ", "low");
                if (Math.abs(Math.abs(slide_left.getCurrentPosition() - LOWPOS)) < 10 ) {
                    back_left.setPower(0);
                    back_right.setPower(0);
                    front_left.setPower(0);
                    front_right.setPower(0);
                    slide_left.setPower(0.04);
                    slide_right.setPower(0.04);
                    //code to flip v4b to junction
                    arm_left.setPower(-0.6);
                    arm_right.setPower(-0.6);
                    v4bTimer.reset();
                    liftTurretState = LiftTurretState.LIFT_FLIPJUNCTION;
                }
                break;
            case LIFT_MID:
                telemetry.addData("state: ", "mid");
                //wait for lift to finish extending
                if (Math.abs(Math.abs(slide_left.getCurrentPosition() - MIDPOS)) < 10 ) {
                    back_left.setPower(0);
                    back_right.setPower(0);
                    front_left.setPower(0);
                    front_right.setPower(0);
                    slide_left.setPower(0.04);
                    slide_right.setPower(0.04);
                    //code to flip v4b to junction
                    arm_left.setPower(-0.6);
                    arm_right.setPower(-0.6);
                    v4bTimer.reset();
                    liftTurretState = LiftTurretState.LIFT_FLIPJUNCTION;
                }
                break;
            case LIFT_HIGH:
                telemetry.addData("state: ", "high");
                //wait for lift to finish extending
                if (Math.abs(Math.abs(slide_left.getCurrentPosition() - HIGHPOS)) < 10 ){
                    back_left.setPower(0);
                    back_right.setPower(0);
                    front_left.setPower(0);
                    front_right.setPower(0);
                    slide_left.setPower(0.04);
                    slide_right.setPower(0.04);
                    //code to flip v4b to junction
                    arm_left.setPower(-0.6);
                    arm_right.setPower(-0.6);
                    v4bTimer.reset();
                    liftTurretState = LiftTurretState.LIFT_FLIPJUNCTION;
                }
                break;
            case LIFT_FLIPJUNCTION:
                telemetry.addData("state: ", "flip junction");
                //wait for amt of time (v4b done moving)
                if (v4bTimer.seconds() >= FLIP_TIME){
                    //done waiting for flip, power off arms
                    arm_left.setPower(0);
                    arm_right.setPower(0);
                }
                if(clawOpen) { //check if claw is opened
                    //start flipping v4b back to middle
                    arm_left.setPower(0.6);
                    arm_right.setPower(0.6);
                    v4bTimer.reset();
                    liftTurretState = LiftTurretState.LIFT_FLIPBACK;
                }
                break;
            case LIFT_FLIPBACK:
                telemetry.addData("state: ", "flipback");
                //wait for amt of time (v4b done moving)
                if (v4bTimer.seconds() >= FLIP_TIME){
                    //done waiting for flip, power off arms
                    arm_left.setPower(0);
                    arm_right.setPower(0);
                    //code to retract lift
                    encoderSet();
                    slide_left.setTargetPosition(BOTTOMPOS);
                    slide_right.setTargetPosition(BOTTOMPOS);
                    slide_left.setPower(-0.2);
                    slide_right.setPower(-0.2);
                    liftTurretState = LiftTurretState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                telemetry.addData("state: ", "retract");
                //wait for done retract
                if (Math.abs(slide_left.getCurrentPosition() - BOTTOMPOS) < 10 ) {
                    liftTurretState = liftTurretState.LIFT_START;
                }
                break;
            case LIFT_FLIPMID:
                telemetry.addData("state: ", "flipmid");
                if (v4bTimer.seconds() >= FLIP_TIME){
                    arm_left.setPower(0);
                    arm_right.setPower(0);
                    liftTurretState = LiftTurretState.LIFT_START;
                }
            default: //should never be reached
                liftTurretState = LiftTurretState.LIFT_START;
        }
        //reset button
        if(gamepad2.left_bumper && liftTurretState != LiftTurretState.LIFT_START){
            liftTurretState = LiftTurretState.LIFT_START;
        }
*/
        double forward = -gamepad1.left_stick_y;
        //    double strafe = gamepad1.right_stick_x;
        double rotate = gamepad1.right_stick_x;
        double slides = -gamepad2.right_stick_y;
        double arms = -gamepad2.left_stick_y;

        double bLPower = forward + rotate; // - strafe
        double bRPower = forward - rotate; //+ strafe
        double fLPower = forward + rotate; //+ strafe
        double fRPower = forward - rotate; // - strafe
        setDrivePowers(bLPower, bRPower, fLPower, fRPower);

        if(liftTurretState == LiftTurretState.LIFT_START && arms > 0.2) {
            arm_left.setPower(0.5);
            arm_right.setPower(0.5);
        }
        else if(liftTurretState == LiftTurretState.LIFT_START && arms < -0.2) {
            arm_left.setPower(-0.5);
            arm_right.setPower(-0.5);
        }
        else if (liftTurretState == LiftTurretState.LIFT_START){
            arm_left.setPower(0);
            arm_right.setPower(0);
        }
        if(liftTurretState == LiftTurretState.LIFT_START) {
            slide_left.setPower(slides + f);
            slide_right.setPower(slides + f);
        }
        if (gamepad2.dpad_left){
            clawOpen = true;
            claw.setPosition(OPEN_CLAW);
        }
        else if (gamepad2.dpad_right && clawOpen){
            clawOpen = false;
            claw.setPosition(CLOSED_CLAW);
        }
        telemetry.addData("l pos", slide_left.getCurrentPosition());
        telemetry.addData("r pos", slide_right.getCurrentPosition());
        telemetry.addData("claw pos ", claw.getPosition());
        telemetry.addData("claw state ", clawOpen);

    }
}




