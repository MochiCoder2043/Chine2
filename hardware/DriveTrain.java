package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {

    private OpMode op;


    public DriveTrain(OpMode op) {
        this.op = op;
    }


    DcMotor lf, rf, lb, rb;
    IMU imu;

    // PID for heading hold
    double kP = 0.0001;
    double targetHeading = 0;
    boolean headingLock = false;

    public void init (HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(params);

        lf = hardwareMap.get(DcMotorEx.class, "LFMotor");
        rf = hardwareMap.get(DcMotorEx.class, "RFMotor");
        lb = hardwareMap.get(DcMotorEx.class, "LBMotor");
        rb = hardwareMap.get(DcMotorEx.class, "RBMotor");

        lf.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void drive(double x, double y, double turn, boolean lockButton) {

        // Mecanum math
        // Handle heading lock toggle
        if (lockButton && !headingLock) {
            headingLock = true;
            targetHeading = getHeading();
        }
        if (!lockButton) {
            headingLock = false;
        }

        // Apply heading correction when locked
        if (headingLock) {
            double error = angleError(targetHeading, getHeading());
            double correction = kP * error;
            turn += correction;
        }

        double lfPower = y + x + turn;
        double rfPower = y - x - turn;
        double lbPower = y - x + turn;
        double rbPower = y + x - turn;

        // Normalize (so we don't go above 1.0)
        double max = Math.max(1.0, Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower), Math.max(Math.abs(lbPower), Math.abs(rbPower)))));
        lfPower /= max;
        rfPower /= max;
        lbPower /= max;
        rbPower /= max;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }


    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // normalize angle error to [-180,180] so PID behaves correctly
    public double angleError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    public String getStatus() {
        return String.format(
                "LeftFront: %.2f\n | LeftBack: %.2f\n | RightFront: %.2f\n | RightBack: %.2f\nHeading: %.2f",
                lf.getPower(),
                lb.getPower(),
                rf.getPower(),
                rb.getPower(),
                targetHeading

        );
    }
}
