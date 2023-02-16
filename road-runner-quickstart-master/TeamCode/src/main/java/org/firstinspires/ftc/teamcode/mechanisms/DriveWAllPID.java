package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

public class DriveWAllPID {

    private PIDController BLcontroller;
    private PIDController BRcontroller;
    private PIDController FLcontroller;
    private PIDController FRcontroller;

    public  double BLp = 0.02, BLi = 0, BLd = 0;
    public  double BRp = 0.02, BRi = 0, BRd = 0;
    public  double FLp = 0.02, FLi = 0, FLd = 0;
    public  double FRp = 0.02, FRi = 0, FRd = 0;

    private   int target = 0;

    private DcMotorEx back_left;
    private DcMotorEx back_right;
    private DcMotorEx front_left;
    private DcMotorEx front_right;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public DriveWAllPID(HardwareMap hwMap) {
        back_left = hwMap.get(DcMotorEx.class, "back_left");
        back_right = hwMap.get(DcMotorEx.class, "back_right");
        front_left = hwMap.get(DcMotorEx.class, "front_left");
        front_right = hwMap.get(DcMotorEx.class, "front_right");
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderFunction();
        BLcontroller = new PIDController(BLp, BLi, BLd);
        BRcontroller = new PIDController(BRp, BRi, BRd);
        FLcontroller = new PIDController(FLp, FLi, FLd);
        FRcontroller = new PIDController(FRp, FRi, FRd);
    }
    public void setTarget(double t){
        target = (int)(t * COUNTS_PER_INCH * 30.0 / 30.75 + 0.5);
    }
    public void move(double correction){
        BLcontroller.setPID(BLp, BLi, BLd);
        BRcontroller.setPID(BRp, BRi, BRd);
        FLcontroller.setPID(FLp, FLi, FLd);
        FRcontroller.setPID(FRp, FRi, FRd);
        int backLPos = back_left.getCurrentPosition();
        double BLpid = BLcontroller.calculate(backLPos, target);
        int backRPos = back_right.getCurrentPosition();
        double BRpid = BRcontroller.calculate(backRPos, target);
        int frontLPos = front_left.getCurrentPosition();
        double FLpid = FLcontroller.calculate(frontLPos, target);
        int frontRPos = front_right.getCurrentPosition();
        double FRpid = FRcontroller.calculate(frontRPos, target);

        double BLpower = BLpid;
        double BRpower = BRpid;
        double FLpower = FLpid;
        double FRpower = FRpid;

        back_left.setPower(BLpower - correction);
        back_right.setPower(BRpower + correction);
        front_left.setPower(FLpower - correction);
        front_right.setPower(FRpower + correction);
        telemetry.addData("bl pos ", backLPos);
        telemetry.addData("target ", target);
        telemetry.addData("error ", target - backLPos);
        telemetry.update();
    }
    public void brake(){
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }
    public void encoderFunction(){
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
