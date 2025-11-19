package org.firstinspires.ftc.teamcode.hardware;

public class PIDController {
    private double kP, kI, kD;

    private double integral = 0;
    private double lastError = 0;

    public PIDController(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public double calculate(double target, double current) {
        double error = target - current;

        integral += error;
        double derivative = error - lastError;

        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    //this is for testing

}
