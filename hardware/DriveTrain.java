package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {

    // Motors (exposed for telemetry if needed)
    private DcMotorEx lf, rf, lb, rb;
    private IMU imu;

    // === Drive tuning ===
    private final double deadband = 0.05;     // joystick deadzone
    private final double strafeScale = 1.05;  // slightly boost strafe if needed
    private final double rampRate = 0.25;     // max power change per update (simple slew)

    // === Heading hold variables ===
    private boolean headingLock = false;
    private double targetHeading = 0.0;
    private final PIDController headingPID = new PIDController(0.012, 0.0, 0.002); // starting gains
    private final double headingDeadbandDeg = 0.8; // small deadband for heading

    // smoothing unit-vector for heading to avoid wrap issues
    private double smoothVx = 1.0;
    private double smoothVy = 0.0;
    private final double headingSmoothAlpha = 0.35; // 0.1..0.5

    // for ramping
    private double prevLf = 0, prevRf = 0, prevLb = 0, prevRb = 0;

    // toggle debounce (OpMode calls drive each loop and passes button)
    private double lastToggleTimeMs = -1000;
    private final double toggleDebounceMs = 250.0;

    public DriveTrain() {
        // empty ctor; init with init(hardwareMap)
    }

    /** Initialize hardware; call from OpMode.init() */
    public void init(HardwareMap hw) {
        imu = hw.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);

        lf = hw.get(DcMotorEx.class, "LFMotor");
        rf = hw.get(DcMotorEx.class, "RFMotor");
        lb = hw.get(DcMotorEx.class, "LBMotor");
        rb = hw.get(DcMotorEx.class, "RBMotor");

        // motor directions - adjust if your wiring is different
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        headingPID.setIntegralLimit(50.0); // protect integral
    }

    /**
     * Main drive call — call from your OpMode.loop().
     *
     * @param forward  forward power from -1..1 (positive = forward)
     * @param strafe   strafe power from -1..1 (positive = right)
     * @param turn     rotation power from -1..1 (positive = clockwise)
     * @param lockBtn  boolean button state used to toggle heading lock
     * @param nowMs    current time in milliseconds (pass runtime.milliseconds())
     * @param dt       delta seconds since last update (pass from ElapsedTime)
     */
    public void drive(double forward, double strafe, double turn, boolean lockBtn, double nowMs, double dt) {
        // 1) apply deadband and simple scaling
        forward = applyDeadband(forward);
        strafe = applyDeadband(strafe) * strafeScale;
        turn = applyDeadband(turn);

        // 2) handle toggle (press-to-toggle). debounce using nowMs
        if (lockBtn && (nowMs - lastToggleTimeMs) > toggleDebounceMs) {
            headingLock = !headingLock;
            lastToggleTimeMs = nowMs;
            if (headingLock) {
                targetHeading = getRawHeading();
                headingPID.reset();
            }
        }

        // 3) read and smooth heading (vector smoothing)
        double measured = getRawHeading(); // -180..180
        double rad = Math.toRadians(measured);
        double vx = Math.cos(rad), vy = Math.sin(rad);
        smoothVx = headingSmoothAlpha * vx + (1 - headingSmoothAlpha) * smoothVx;
        smoothVy = headingSmoothAlpha * vy + (1 - headingSmoothAlpha) * smoothVy;
        double smoothHeading = Math.toDegrees(Math.atan2(smoothVy, smoothVx));

        // 4) optionally apply heading PID (before kinematics)
        if (headingLock) {
            double err = angleDiff(targetHeading, smoothHeading);
            if (Math.abs(err) < headingDeadbandDeg) err = 0;
            double correction = headingPID.update(err, dt);
            // clamp correction to avoid overpowering driver input
            correction = Range.clip(correction, -0.35, 0.35);
            turn += correction;
        }

        // 5) mecanum kinematics
        double lfPower = forward + strafe + turn;
        double rfPower = forward - strafe - turn;
        double lbPower = forward - strafe + turn;
        double rbPower = forward + strafe - turn;

        // 6) normalize
        double max = Math.max(1.0, Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower), Math.max(Math.abs(lbPower), Math.abs(rbPower)))));
        lfPower /= max;
        rfPower /= max;
        lbPower /= max;
        rbPower /= max;

        // 7) ramping (simple slew rate)
        lfPower = ramp(prevLf, lfPower, rampRate);
        rfPower = ramp(prevRf, rfPower, rampRate);
        lbPower = ramp(prevLb, lbPower, rampRate);
        rbPower = ramp(prevRb, rbPower, rampRate);

        // store for next loop
        prevLf = lfPower; prevRf = rfPower; prevLb = lbPower; prevRb = rbPower;

        // 8) apply to motors
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    // helper: return raw imu yaw in degrees [-180,180]
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // helper: normalized shortest difference target - current in [-180,180]
    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    private double applyDeadband(double v) {
        return Math.abs(v) < deadband ? 0.0 : v;
    }

    private double ramp(double prev, double wanted, double step) {
        double diff = wanted - prev;
        if (Math.abs(diff) <= step) return wanted;
        return prev + Math.signum(diff) * step;
    }

    // telemetry-friendly status
    public String getStatus() {
        return String.format("LF: %.2f LFenc: %d\nRF: %.2f RFenc: %d\nLB: %.2f LBenc: %d\nRB: %.2f RBenc: %d\nHLock:%b Ht:%.1f",
                lf.getPower(), lf.getCurrentPosition(),
                rf.getPower(), rf.getCurrentPosition(),
                lb.getPower(), lb.getCurrentPosition(),
                rb.getPower(), rb.getCurrentPosition(),
                headingLock, targetHeading
        );
    }

    // ==== Inner PID controller (simple, robust) ====
    private static class PIDController {
        private final double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double integralLimit = Double.POSITIVE_INFINITY;

        public PIDController(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d;
        }

        public void setIntegralLimit(double limit) { this.integralLimit = Math.abs(limit); }

        public double update(double error, double dt) {
            if (dt <= 0) {
                lastError = error;
                return kP * error;
            }
            integral += error * dt;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;
            double derivative = (error - lastError) / dt;
            lastError = error;
            return kP * error + kI * integral + kD * derivative;
        }

        public void reset() {
            integral = 0; lastError = 0;
        }
    }
}
