package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "AprilTag Detection Example")
public class AprilTagDetectionExample extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // Connect the camera
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Starting AprilTag detection...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine(String.format("Tag ID: %d", detection.id));
                    telemetry.addLine(String.format("X: %.1f cm", detection.ftcPose.x));
                    telemetry.addLine(String.format("Y: %.1f cm", detection.ftcPose.y));
                    telemetry.addLine(String.format("Z: %.1f cm", detection.ftcPose.z));
                    telemetry.addLine(String.format("Yaw: %.1f deg", detection.ftcPose.yaw));
                    telemetry.addLine(String.format("Pitch: %.1f deg", detection.ftcPose.pitch));
                    telemetry.addLine(String.format("Roll: %.1f deg", detection.ftcPose.roll));
                    telemetry.addLine("----------------");
                }
            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}
