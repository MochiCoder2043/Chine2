package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@TeleOp(name="Jack1", group="Iterative OpMode")
public class Jack extends OpMode {

    private DriveTrain drive = new DriveTrain();

    @Override
    public void init() {

        drive.init(hardwareMap);
        telemetry.addLine("DriveTrain initialized");

    }

    @Override
    public void start() {
        telemetry.addLine("Starting TeleOp");
        telemetry.update();
    }

    @Override
    public void loop() {

        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_trigger - gamepad1.left_trigger;
        double twist = gamepad1.right_stick_x;

        drive.drive(power, strafe, twist);

        telemetry.addData("Drive", drive.getStatus());
        telemetry.update();

    }
}
