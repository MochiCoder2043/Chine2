package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Pelvis;


@TeleOp(name="Jack", group="Iterative OpMode")
public class Jack extends OpMode {

    private DriveTrain drive = new DriveTrain();
    private Pelvis pelvis = new Pelvis();
    AprilTagDetector detector = new AprilTagDetector();
    private ElapsedTime runtime = new ElapsedTime();

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
        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_trigger - gamepad1.left_trigger;
        double twist = gamepad1.right_stick_x;

        // Gamepad inputs for throwing the ball
        double pelvisInput = gamepad2.left_stick_y;
        double intakePower = gamepad2.right_stick_y;

        drive.drive(power, strafe, twist);
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
