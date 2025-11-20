package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetector {

    private VisionPortal portal;
    private AprilTagProcessor processor;

    public void init(HardwareMap hardwareMap) {

        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagID(false)
                .setDrawCubeProjection(false)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();
    }

    /**
     * Returns a READY-TO-USE object with guaranteed values.
     * Never returns null. Never requires if-statements.
     */
    public TagResult getResult() {
        List<AprilTagDetection> list = processor.getDetections();

        if (list.size() > 0) {
            AprilTagDetection d = list.get(0);
            return new TagResult(
                    true,
                    d.id,
                    d.ftcPose.x,
                    d.ftcPose.y,
                    d.ftcPose.z,
                    d.ftcPose.yaw,
                    d.ftcPose.pitch,
                    d.ftcPose.roll
            );
        }

        // no tag → return a SAFE result with found=false
        return new TagResult(false, -1, 0, 0, 0, 0, 0, 0);
    }

    public void close() {
        portal.close();
    }

    // ================================
    // The result object you will use
    // ================================
    public static class TagResult {
        public boolean found;
        public int id;
        public double x, y, z;
        public double yaw, pitch, roll;

        public TagResult(boolean found, int id,
                         double x, double y, double z,
                         double yaw, double pitch, double roll) {

            this.found = found;
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
        }
    }
}
