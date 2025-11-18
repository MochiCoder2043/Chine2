package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private final int BASEPOWER = 2500;

    public void init (HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "LFMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "LBMotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "RBMotor");

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double power, double strafe, double twist) {

        double LFPower = Range.clip(power + strafe + twist, -1.0, 1.0);
        double LBPower = Range.clip(power - strafe + twist, -1.0, 1.0);
        double RFPower = Range.clip(power - strafe - twist, -1.0, 1.0);
        double RBPower = Range.clip(power + strafe - twist, -1.0, 1.0);

        leftFront.setVelocity(LFPower*BASEPOWER);
        leftBack.setVelocity(LBPower*BASEPOWER);
        rightFront.setVelocity(RFPower*BASEPOWER);
        rightBack.setVelocity(RBPower*BASEPOWER);
    }

    public String getStatus() {
        return String.format(
                "LF: %.0f | RF: %.0f",
                leftFront.getVelocity(),
                rightFront.getVelocity()
        );
    }
}
