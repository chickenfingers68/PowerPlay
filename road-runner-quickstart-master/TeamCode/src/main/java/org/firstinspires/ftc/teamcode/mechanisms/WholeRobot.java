package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WholeRobot {
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private CRServo arm_left;
    private CRServo arm_right;
    private Servo claw;

    public void init(HardwareMap hwMap) {
        back_left = hwMap.get(DcMotor.class, "back_left");
        back_right = hwMap.get(DcMotor.class, "back_right");
        front_left = hwMap.get(DcMotor.class, "front_left");
        front_right = hwMap.get(DcMotor.class, "front_right");
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm_left = hwMap.get(CRServo.class, "arm_left");
        arm_right = hwMap.get(CRServo.class, "arm_right");
        arm_left.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hwMap.get(Servo.class, "claw");

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

    public void drive(double forward, double strafe, double rotate) {
        double bLPower = forward - strafe + rotate;
        double bRPower = forward + strafe - rotate;
        double fLPower = forward + strafe + rotate;
        double fRPower = forward - strafe - rotate;
        setDrivePowers(bLPower, bRPower, fLPower, fRPower);
    }

    public void setArmPowers(double armPower) {
        arm_left.setPower(0.7 * armPower);
        arm_right.setPower(0.7 * armPower);
    }
    public double getClawPosition(){
        return claw.getPosition();
    }
    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
}
