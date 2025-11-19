package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="autoJack")
public class AutoJack extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private DriveTrain drive;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopTimeSec = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain();
        drive.init(hardwareMap);
        telemetry.addLine("Drive initialized");
        telemetry.update();
        lastLoopTimeSec = runtime.seconds();

    }
}
