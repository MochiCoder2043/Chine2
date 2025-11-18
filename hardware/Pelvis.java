package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pelvis {

    private DcMotorEx rightPelvis, leftPelvis, launch;
    private DcMotor pickUp;

    private final int MAX_LAUNCH_VELOCITY = 2500;

    public void init(HardwareMap hardwareMap) {

        pickUp = hardwareMap.get(DcMotor.class,"pickUp");
        rightPelvis = hardwareMap.get(DcMotorEx.class, "rightPelvis");
        leftPelvis = hardwareMap.get(DcMotorEx.class, "leftPelvis");
        launch = hardwareMap.get(DcMotorEx.class, "launch");

        // Directions for the throwing motors
        leftPelvis.setDirection(DcMotorEx.Direction.FORWARD);
        rightPelvis.setDirection(DcMotorEx.Direction.REVERSE);
        pickUp.setDirection(DcMotorEx.Direction.REVERSE);


        leftPelvis.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightPelvis.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftPelvis.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightPelvis.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void launch(double pelvisInput, double intakePower) {

        double targetVelocity = pelvisInput * MAX_LAUNCH_VELOCITY;

        leftPelvis.setVelocity(targetVelocity);
        rightPelvis.setVelocity(targetVelocity);
        launch.setVelocity(targetVelocity);
        pickUp.setPower(intakePower);

    }

    public String getStatus() {
        return String.format(
                "leftPelvis: %.0f | rightPelvis: %.0f | launch: %.0f | pickUp: %.0f",
                leftPelvis.getVelocity(),
                rightPelvis.getVelocity(),
                launch.getVelocity(),
                pickUp.getPower()
        );
    }
}
