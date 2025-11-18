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
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        private DcMotorEx LBMotor = null;
        private DcMotorEx RBMotor = null;
        private DcMotorEx LFMotor = null;
        private DcMotorEx RFMotor = null;

        // The motors for the ball throwing bloody thing
        // TODO: Read documentation to change to ideal motor for speed
        private DcMotor pickUp = null;
        private DcMotorEx launch = null;

        // You are not allowed to judge I am sleep deprived
        private DcMotorEx rightPelvis = null;
        private DcMotorEx leftPelvis = null;
        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
                telemetry.addData("Status", "Initialized");

                // Initialize the hardware variables. Note that the strings used here as parameters
                // to 'get' must correspond to the names assigned during the robot configuration
                // step (using the FTC Robot Controller app on the phone).
                LBMotor  = hardwareMap.get(DcMotorEx.class, "LBMotor");
                RBMotor  = hardwareMap.get(DcMotorEx.class, "RBMotor");
                LFMotor  = hardwareMap.get(DcMotorEx.class, "LFMotor");
                RFMotor  = hardwareMap.get(DcMotorEx.class, "RFMotor");

                // Initializing the Motors to the correct entry
                pickUp = hardwareMap.get(DcMotor.class,"pickUp");
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


        leftPelvis.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightPelvis.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftPelvis.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightPelvis.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

        double pelvisInput = gamepad2.left_stick_y;
        double intakePower = gamepad2.right_stick_y;

        // Scale to your desired maximum velocity
       // This is now your actual max speed
        double maxLaunchVelocity = 2500;
        double targetVelocity = pelvisInput * maxLaunchVelocity;

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_trigger - gamepad1.left_trigger;
        //double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;
        double BasePower = 2500;

        // TODO: Set up classes for everything

        double LFPower = Range.clip(drive + strafe + twist, -1.0, 1.0);
        double RFPower = Range.clip(drive - strafe - twist, -1.0, 1.0);
        double LBPower = Range.clip(drive - strafe + twist, -1.0, 1.0);
        double RBPower = Range.clip(drive + strafe - twist, -1.0, 1.0);

        LFMotor.setVelocity(LFPower*BasePower);
        RFMotor.setVelocity(RFPower*BasePower);
        LBMotor.setVelocity(LBPower*BasePower);
        RBMotor.setVelocity(RBPower*BasePower);

        leftPelvis.setVelocity(targetVelocity);
        rightPelvis.setVelocity(targetVelocity);
        launch.setVelocity(targetVelocity);
        pickUp.setPower(intakePower);


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
