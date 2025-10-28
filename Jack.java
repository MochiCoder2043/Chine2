package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Jack", group="Iterative OpMode")
public class Jack extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LBMotor = null;
    private DcMotor RBMotor = null;
    private DcMotor LFMotor = null;
    private DcMotor RFMotor = null;

    // The motors for the ball throwing bloody thing
    // TODO: Read documentation to change to ideal motor for speed
    private DcMotor pickUp = null;
    private DcMotor launch = null;

    // You are not allowed to judge I am sleep deprived
    private DcMotor rightPelvis = null;
    private DcMotor leftPelvis = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LBMotor  = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor  = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RFMotor");

        // Initializing the Motors to the correct entry
        pickUp = hardwareMap.get(DcMotor.class,"pickUp");
        rightPelvis = hardwareMap.get(DcMotor.class, "rightPelvis");
        leftPelvis = hardwareMap.get(DcMotor.class, "leftPelvis");
        launch = hardwareMap.get(DcMotor.class, "launch");


        // TODO: set up the motors with forward and reverse


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);

        // Directions for the thorughing motors
        leftPelvis.setDirection(DcMotor.Direction.FORWARD);
        rightPelvis.setDirection(DcMotor.Direction.REVERSE);
        launch.setDirection(DcMotor.Direction.REVERSE);
        pickUp.setDirection(DcMotor.Direction.FORWARD);

        rightPelvis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPelvis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightPelvis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPelvis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double LBPower;
        double RBPower;
        double LFPower;
        double RFPower;

        // Declaring power for the pushy thing
        double outtakePower;
        double intakePower;
        double launchPower = gamepad2.right_trigger - gamepad2.left_trigger;


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_trigger - gamepad1.left_trigger;
        //double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.
        // TODO: Fix this stupid disgusting bullshit code
        if (gamepad2.left_stick_y > 0.1){
            outtakePower = -(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y < -0.1){
            outtakePower = -(gamepad2.left_stick_y);
        } else {
            outtakePower = 0;
        }

        //separate intake from outake
        if (gamepad2.right_stick_y > 0.1) {
            intakePower = -gamepad2.right_stick_y;
        } else if (gamepad2.right_stick_y < -0.1) {
            intakePower = -gamepad2.right_stick_y;
        } else {
            intakePower = 0;
        }


        pickUp.setPower(intakePower);
        launch.setPower(outtakePower);
        rightPelvis.setPower(outtakePower);
        leftPelvis.setPower(outtakePower);

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // TODO: Set up PID for accurate movement
        // TODO: Set up classes for everything

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;
        LFPower = Range.clip(drive + strafe + twist, -1.0, 1.0);
        RFPower = Range.clip(drive - strafe - twist, -1.0, 1.0);
        LBPower = Range.clip(drive - strafe + twist, -1.0, 1.0);
        RBPower = Range.clip(drive + strafe - twist, -1.0, 1.0);


        LFMotor.setPower(LFPower);
        RFMotor.setPower(RFPower);
        LBMotor.setPower(LBPower);
        RBMotor.setPower(RBPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "LB: %.2f | RB: %.2f | LF: %.2f | RF: %.2f", LBPower, RBPower, LFPower, RFPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    // this is just a simple test to understand branches
    @Override
    public void stop() {
    }

}
