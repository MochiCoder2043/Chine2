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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="autoBeta", group="Robot")
public class autoBeta extends LinearOpMode {

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

    // Calculate the COUNTS_PER_CM for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_CM   = 8.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

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
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                LBMotor.getCurrentPosition(),
                LFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(0.5, 100, 0, 0, 5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double driveCm, double strafeCm, double turnCm,
                             double timeoutS) {
        int newLBTarget;
        int newRBTarget;
        int newLFTarget;
        int newRFTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLBTarget = LBMotor.getCurrentPosition() + (int)((driveCm-strafeCm+turnCm) * COUNTS_PER_CM);
            newRBTarget = RBMotor.getCurrentPosition() + (int)((driveCm+strafeCm-turnCm) * COUNTS_PER_CM);
            newLFTarget = LFMotor.getCurrentPosition() + (int)((driveCm+strafeCm+turnCm) * COUNTS_PER_CM);
            newRFTarget = RFMotor.getCurrentPosition() + (int)((driveCm-strafeCm-turnCm) * COUNTS_PER_CM);

            LBMotor.setTargetPosition(newLBTarget);
            RBMotor.setTargetPosition(newRBTarget);
            LFMotor.setTargetPosition(newLFTarget);
            RFMotor.setTargetPosition(newRFTarget);


            // Turn On RUN_TO_POSITION
            LBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LBMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));
            LFMotor.setPower(Math.abs(speed));
            RFMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LBMotor.isBusy() && RBMotor.isBusy() && LFMotor.isBusy() && RFMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLBTarget,  newRBTarget, newLFTarget,  newRFTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d ",
                        LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), RFMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LBMotor.setPower(0);
            RBMotor.setPower(0);
            LFMotor.setPower(0);
            RFMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LBMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            LFMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
