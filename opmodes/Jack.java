package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Pelvis;


@TeleOp(name="Jack", group="Iterative OpMode")
public class Jack extends OpMode {

    private Pelvis pelvis = new Pelvis();
    AprilTagDetector detector = new AprilTagDetector();
    private ElapsedTime runtime = new ElapsedTime();

    private DriveTrain drive = new DriveTrain(this);




    @Override
    public void init() {

        drive.init(hardwareMap);
        pelvis.init(hardwareMap);
        detector.init(hardwareMap);

        telemetry.addLine("DriveTrain initialized");

    }

    @Override
    public void start() {
        telemetry.addLine("Starting TeleOp");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Gamepad inputs for the chasis movement

        // Drivetrain input
        double y = -gamepad1.left_stick_y; // forward/back
        double x = (gamepad1.right_trigger - gamepad1.left_trigger) * 1.1; // strafe (boosted)
        double turn = gamepad1.right_stick_x;   // turning power

        // Toggle heading lock (press X)
        boolean lock = gamepad1.x;


        // Gamepad inputs for throwing the ball
        double pelvisInput = gamepad2.left_stick_y;
        double intakePower = gamepad2.right_stick_y;

        drive.drive(x, y, turn, lock);
        pelvis.launch(pelvisInput, intakePower);
        AprilTagDetector.TagResult tag = detector.getResult();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive", drive.getStatus());
        telemetry.addData("Pelvis", pelvis.getStatus());
        // april tag data
        telemetry.addData("Found", tag.found);
        telemetry.addData("ID", tag.id);
        telemetry.addData("X", tag.x);
        telemetry.addData("Y", tag.y);
        telemetry.addData("Z", tag.z);
        telemetry.addData("Yaw", tag.yaw);
        telemetry.addData("Pitch", tag.pitch);
        telemetry.addData("Roll", tag.roll);

        telemetry.update();

    }
}
