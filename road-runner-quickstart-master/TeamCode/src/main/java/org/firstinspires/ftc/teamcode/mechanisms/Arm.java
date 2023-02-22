package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private CRServo arm_left;
    private CRServo arm_right;
    private double power;

    public Arm(HardwareMap hwMap){
        arm_left = hwMap.get(CRServo.class, "slide_left");
        arm_right = hwMap.get(CRServo.class, "slide_right");

        arm_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update(){
        arm_left.setPower(power);
        arm_right.setPower(power);
    }
    public void idle(){
        arm_right.setPower(0.03);
        arm_left.setPower(0.03);
    }

    public void setPower(double power){
        this.power = power;
    }
}
