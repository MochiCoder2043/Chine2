package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Pelvis;

@TeleOp(name = "Jack", group = "Main")
public class Jack extends OpMode {

    private DriveTrain drive;
    private Pelvis shakira;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopTimeSec = 0.0;
    AprilTagDetector tagDetector = new AprilTagDetector();


    @Override
    public void init() {

        tagDetector.init(hardwareMap);

        drive = new DriveTrain();
        drive.init(hardwareMap);

        shakira = new Pelvis();
        shakira.init(hardwareMap);

        telemetry.addLine("Drive initialized");
        telemetry.update();
        lastLoopTimeSec = runtime.seconds();
    }

    @Override
    public void loop() {

        AprilTagDetector.TagResult tag = tagDetector.getResult();

        // calculate dt (seconds) and now ms
        double nowSec = runtime.seconds();
        double dt = nowSec - lastLoopTimeSec;
        if (dt <= 0) dt = 0.001;
        lastLoopTimeSec = nowSec;

        double nowMs = runtime.milliseconds();

        // read gamepad
        double forward = -gamepad1.left_stick_y;   // forward positive
        double strafe  =  gamepad1.right_trigger - gamepad1.left_trigger;   // right positive
        double turn    =  gamepad1.right_stick_x;  // clockwise positive

        boolean lockBtn = gamepad1.x; // press X to toggle heading lock

        double pelvisInput = gamepad2.left_stick_y;
        double intakePower = gamepad2.right_stick_y;

        shakira.launch(pelvisInput, intakePower);


        // feed into drivetrain (non-blocking)
        drive.drive(forward, strafe, turn, lockBtn, nowMs, dt);

        // telemetry (small, useful)
        telemetry.addData("Heading", String.format("%.1f", drive.getRawHeading()));
        telemetry.addData("Status", drive.getStatus());
        telemetry.addData("Distance X (cm)", "%.1f", tag.x);
        telemetry.addData("Distance Y (cm)", "%.1f", tag.y);
        telemetry.addData("Distance Z (cm)", "%.1f", tag.z);
        telemetry.addData("Id:", "%s", tag.id);
        telemetry.update();
    }
}
