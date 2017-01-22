/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Timer;

@Disabled
@Autonomous(name="Shooter Autonomous")
public class ShooterAutonomousTest extends LinearOpMode {

    /* Declare OpMode members. */
    ShooterBot robot   = new ShooterBot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.2;
    static final double     PI                      = 3.1415;   // pi!
    static final double     WHEEL_DIST_INCHES       = 14.25;    // Distance between the wheels
    static final int        PAUSE_MOVEMENT          = 250;      // pause between each movement
    static final int        LONG_PAUSE_MOVEMENT     = 1000;      // pause between each movement
    static final int        NO_PAUSE_MOVEMENT       = 0;      // pause between each movement

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
        idle();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(0.05,  46,  46, 10.0, PAUSE_MOVEMENT);  // S1: forward 48 inches with 10 sec timeout
        robot.leftMotor.setPower(1);
        robot.rightMotor.setPower(1);
        try {
            Thread.sleep(100);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        shoot();
        try {
            Thread.sleep(1000);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }

        robot.leftMotor.setPower(-0.2);
        robot.rightMotor.setPower(-0.2);
        try {
            Thread.sleep(100);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftMotor.setPower(0.2);
        robot.rightMotor.setPower(0.2);
        try {
            Thread.sleep(100);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        shoot();

        robot.leftMotor.setPower(0.05);
        robot.rightMotor.setPower(0.05);
        try {
            Thread.sleep(1500);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setPower(-0.2);
        robot.rightMotor.setPower(-0.2);
        try {
            Thread.sleep(200);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, int movementPause) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(-Math.abs(newLeftTarget));
            robot.rightMotor.setTargetPosition(-Math.abs(newRightTarget));
//
//            // Turn On RUN_TO_POSITION
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", -newLeftTarget,  -newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(movementPause);   // pause after each move so that movements are more accurate
        }
    }
    public void turnDrive(double speed, double angle, double timeoutS, int movementPause) throws InterruptedException {
        double arcLength = WHEEL_DIST_INCHES * PI / 360 * angle;
        encoderDrive(speed, arcLength, -arcLength, timeoutS, movementPause);
    }
    public void shoot() throws InterruptedException {
        // TODO copied from ShooterDrivetrain.java
        robot.leftWheel.setPower(-0.1);
        robot.rightWheel.setPower(0.1);
        try {
            Thread.sleep(50);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftWheel.setPower(-0.2);
        robot.rightWheel.setPower(0.2);
        try {
            Thread.sleep(50);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftWheel.setPower(-0.3);
        robot.rightWheel.setPower(0.3);
        try {
            Thread.sleep(50);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftWheel.setPower(-0.4);
        robot.rightWheel.setPower(0.4);
        try {
            Thread.sleep(50);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftWheel.setPower(-0.5);
        robot.rightWheel.setPower(0.5);
        try {
            Thread.sleep(50);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftWheel.setPower(-0.6);
        robot.rightWheel.setPower(0.6);
        try {
            Thread.sleep(50);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.leftWheel.setPower(-0.7);
        robot.rightWheel.setPower(0.7);
        try {
            Thread.sleep(500);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
//        robot.leftWheel.setPower(-0.8);
//        robot.rightWheel.setPower(0.8);
//        try {
//            Thread.sleep(500);
//        }
//        catch (java.lang.InterruptedException e) {
//            telemetry.addData("error", e);
//        }
        robot.piston.setPower(-1);
        try {
            Thread.sleep(1800);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.piston.setPower(1);
        try {
            Thread.sleep(1800);
        }
        catch (java.lang.InterruptedException e) {
            telemetry.addData("error", e);
        }
        robot.piston.setPower(0);
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        }
    }
