package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private PIDController controller;

    public  double p = 0.006, i = 0, d = 0;
    public  double f = 0.04;

    private   int target = 0;

    private DcMotorEx slide_left;
    private DcMotorEx slide_right;

    public Lift(HardwareMap hwMap){
        slide_left = hwMap.get(DcMotorEx.class, "slide_left");
        slide_right = hwMap.get(DcMotorEx.class, "slide_right");

        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDController(p, i, d);
    }
    public void setTarget(int t){
        target = t;
    }
    public int getTarget(){
        return target;
    }
    public void update(){
        controller.setPID(p, i, d);
        int slidePosL = slide_left.getCurrentPosition();
        double pid = controller.calculate(slidePosL, target);

        double power = pid + f;

        slide_left.setPower(-power);
        slide_right.setPower(-power);
    }
    public void idle(){
        slide_right.setPower(0.04);
        slide_left.setPower(0.04);
    }
    public void stopReset(){
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setLiftPowers(double power){
        slide_left.setPower(-power);
        slide_right.setPower(-power);
    }
}
