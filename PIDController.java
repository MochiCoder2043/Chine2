package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double kp, ki, kd;
    public double integralSum = 0;
    public double lastError = 0;
    public ElapsedTime timer = new ElapsedTime();
}
