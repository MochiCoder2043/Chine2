package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.AprilTagDetector;

@Autonomous(name = "Auto: Stop 60cm From Tag")
public class AutoJack extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain drive = new DriveTrain();
        AprilTagDetector tagDetector = new AprilTagDetector();

        drive.init(hardwareMap);
        tagDetector.init(hardwareMap);

        // PID for distance
        DistancePID pid = new DistancePID(0.03, 0, 0.001);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Ready - waiting for start");
        telemetry.update();

        waitForStart();
        timer.reset();

        double targetDistance = 53.0;   // centimeters

        while (opModeIsActive()) {

            // Read tag
            AprilTagDetector.TagResult tag = tagDetector.getResult();

            if (!tag.found) {
                // Tag lost → stop robot
                drive.drive(0, 0, 0, false, 0, 0);
                telemetry.addLine("NO TAG FOUND");
                telemetry.update();
                continue;
            }

            // Distance from tag (cm)
            double distance = Math.abs(tag.y);

            // PID update
            double dt = timer.seconds();
            timer.reset();

            double error = distance - targetDistance;  // positive = too far
            double forward = pid.update(error, dt);

            // Limit forward speed
            forward = Math.max(-0.4, Math.min(0.4, forward));

            // If within 2cm → stop
            if (Math.abs(error) < 2.0) {
                forward = 0;
            }

            // Drive forward only (no strafe, no turning)
            drive.drive(forward, 0, 0, false, 0, dt);

            // Telemetry
            telemetry.addData("Tag Found", tag.found);
            telemetry.addData("Distance Z (cm)", "%.1f", tag.z);
            telemetry.addData("Distance Y (cm)", "%.1f", distance);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("PID Output", "%.2f", forward);
            telemetry.addData("status", drive.getStatus());
            telemetry.update();
        }

        tagDetector.close();
    }

    // ======================
    // SIMPLE PID CONTROLLER
    // ======================
    public static class DistancePID {
        double kP, kI, kD;
        double integral = 0;
        double lastError = 0;

        public DistancePID(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
        }

        public double update(double error, double dt) {
            if (dt <= 0) return 0;

            integral += error * dt;
            double derivative = (error - lastError) / dt;

            lastError = error;

            return kP * error + kI * integral + kD * derivative;
        }
    }
}
