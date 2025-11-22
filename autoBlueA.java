package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="auto-blue-A", group="auto-blue")
public class autoBlueA extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx LBMotor = null;
    private DcMotorEx RBMotor = null;
    private DcMotorEx LFMotor = null;
    private DcMotorEx RFMotor = null;
    private DcMotorEx pickUp = null;
    private DcMotorEx launch = null;

    // You are not allowed to judge I am sleep deprived
    private DcMotorEx rightPelvis = null;
    private DcMotorEx leftPelvis = null;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LBMotor = hardwareMap.get(DcMotorEx.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");

        // Initializing the Motors to the correct entry
        pickUp = hardwareMap.get(DcMotorEx.class, "pickUp");
        rightPelvis = hardwareMap.get(DcMotorEx.class, "rightPelvis");
        leftPelvis = hardwareMap.get(DcMotorEx.class, "leftPelvis");
        launch = hardwareMap.get(DcMotorEx.class, "launch");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LBMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RBMotor.setDirection(DcMotorEx.Direction.FORWARD);
        LFMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RFMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Directions for the throwing motors
        leftPelvis.setDirection(DcMotorEx.Direction.FORWARD);
        rightPelvis.setDirection(DcMotorEx.Direction.REVERSE);

        pickUp.setDirection(DcMotorEx.Direction.REVERSE);
        pickUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pickUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftPelvis.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightPelvis.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftPelvis.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightPelvis.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                LBMotor.getCurrentPosition(),
                LFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // FIRST MOVE - 1000 ticks
        RFMotor.setTargetPosition(2000);
        RBMotor.setTargetPosition(2000);
        LFMotor.setTargetPosition(2000);
        LBMotor.setTargetPosition(2000);

        LBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RFMotor.setPower(0.5);
        RBMotor.setPower(0.5);
        LFMotor.setPower(0.5);
        LBMotor.setPower(0.5);

        // WAIT FOR FIRST MOVE TO COMPLETE (FIXED - no Thread.sleep)
        while (opModeIsActive() &&
                (LBMotor.isBusy() || LFMotor.isBusy() || RBMotor.isBusy() || RFMotor.isBusy())) {
            // Wait for motors to reach target position
        }

        // STOP MOTORS
        LBMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        RFMotor.setPower(0);

        // RESET ENCODERS FOR SECOND MOVE
        LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // SET MODES BACK TO RUN_TO_POSITION
        LBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // SECOND MOVE - 2000 ticks
        RFMotor.setTargetPosition(300);
        RBMotor.setTargetPosition(300);
        LFMotor.setTargetPosition(-300);
        LBMotor.setTargetPosition(-300);

        RFMotor.setPower(0.5);
        RBMotor.setPower(0.5);
        LFMotor.setPower(-0.5);
        LBMotor.setPower(-0.5);

        // WAIT FOR SECOND MOVE TO COMPLETE (FIXED - no Thread.sleep)
        while (opModeIsActive() &&
                (LBMotor.isBusy() || LFMotor.isBusy() || RBMotor.isBusy() || RFMotor.isBusy())) {
            // Wait for motors to reach target position
        }

        // STOP MOTORS
        LBMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        RFMotor.setPower(0);

        // RESET ENCODERS FOR SECOND MOVE
        LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // SET MODES BACK TO RUN_TO_POSITION
        LBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // THIRD MOVE - 500 ticks
        RFMotor.setTargetPosition(1000);
        RBMotor.setTargetPosition(1000);
        LFMotor.setTargetPosition(1000);
        LBMotor.setTargetPosition(1000);

        RFMotor.setPower(0.5);
        RBMotor.setPower(0.5);
        LFMotor.setPower(0.5);
        LBMotor.setPower(0.5);

        // WAIT FOR THIRD MOVE TO COMPLETE (FIXED - no Thread.sleep)
        while (opModeIsActive() &&
                (LBMotor.isBusy() || LFMotor.isBusy() || RBMotor.isBusy() || RFMotor.isBusy())) {
            // Wait for motors to reach target position
        }

        // STOP MOTORS
        LBMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        RFMotor.setPower(0);

        // START PELVIS AND LAUNCH MOTORS AT 2500 VELOCITY
        leftPelvis.setVelocity(-2000);
        rightPelvis.setVelocity(-2000);
        launch.setVelocity(-2000);

        // WAIT FOR 3 SECONDS WHILE PELVIS/LAUNCH ACCELERATE
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            // Just wait for 3 seconds
        }

        // START PICKUP MOTOR AT 2500 VELOCITY
        pickUp.setVelocity(-2500);

        // WAIT FOR 1 SECOND WHILE PICKUP ACCELERATES
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.0) {
            // Just wait for 1 second
        }

        // STOP ALL MOTORS
        leftPelvis.setVelocity(0);
        rightPelvis.setVelocity(0);
        launch.setVelocity(0);
        pickUp.setVelocity(0);

        // RESET ENCODERS FOR SECOND MOVE
        LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // SET MODES BACK TO RUN_TO_POSITION
        LBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // THIRD MOVE - 500 ticks
        RFMotor.setTargetPosition(-500);
        RBMotor.setTargetPosition(-500);
        LFMotor.setTargetPosition(-500);
        LBMotor.setTargetPosition(-500);

        RFMotor.setPower(-0.5);
        RBMotor.setPower(-0.5);
        LFMotor.setPower(-0.5);
        LBMotor.setPower(-0.5);

        while (opModeIsActive() &&
                (LBMotor.isBusy() || LFMotor.isBusy() || RBMotor.isBusy() || RFMotor.isBusy())) {
            // Wait for motors to reach target position
        }

        // STOP MOTORS
        LBMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        RFMotor.setPower(0);

        // RESET ENCODERS FOR SECOND MOVE
        LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // SET MODES BACK TO RUN_TO_POSITION
        LBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // SECOND MOVE - 2000 ticks
        RFMotor.setTargetPosition(-500);
        RBMotor.setTargetPosition(-500);
        LFMotor.setTargetPosition(500);
        LBMotor.setTargetPosition(500);

        RFMotor.setPower(-0.5);
        RBMotor.setPower(-0.5);
        LFMotor.setPower(0.5);
        LBMotor.setPower(0.5);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}