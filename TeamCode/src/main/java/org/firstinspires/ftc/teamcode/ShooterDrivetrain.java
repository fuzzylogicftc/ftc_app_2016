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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Shooter Drivetrain")
public class ShooterDrivetrain extends OpMode{

    /* Declare OpMode members. */
    ShooterBot robot = new ShooterBot();

    double lt;
    double rt;
    double leftRaw;
    double rightRaw;
    double leftScaled;
    double rightScaled;
    double HARVESTER_SPEED = 1.0; // speed of the paddle
    boolean harvesterOn = false;
    boolean harvesterBackward = false;
    double variablePower = 0;

     /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Move the robot
        // Left Stick    = Left Motor
        // Right Stick   = Right Motor
        // Left Trigger  = Slow Down
        // Right Trigger = Speed Up

        lt = gamepad1.left_trigger;
        rt = gamepad1.right_trigger;

        leftRaw = gamepad1.left_stick_y;
        rightRaw = gamepad1.right_stick_y;

        leftScaled = scaleInput(leftRaw, lt, rt);
        rightScaled = scaleInput(rightRaw, lt, rt);

        robot.leftMotor.setPower(leftScaled);
        robot.rightMotor.setPower(rightScaled);


        // Control the harvester
        if (gamepad1.left_bumper) {
            // toggle harvesterOn
            harvesterOn = !harvesterOn;
            try {
                Thread.sleep(500);
            }
            catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
        }
        if (gamepad1.right_bumper) {
            harvesterBackward = !harvesterBackward;
            try {
                Thread.sleep(500);
            }
            catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
        }
        if (harvesterOn && !harvesterBackward) {
            robot.harvester.setPower(-HARVESTER_SPEED);
        } else if (harvesterOn) {
            robot.harvester.setPower(0.1);
        } else {
            robot.harvester.setPower(0);
        }


        // Control the shooter
        if (gamepad1.y) {
            // autoshoot

            // for-loops glitch for some reason
            robot.leftWheel.setPower(-0.1);
            robot.rightWheel.setPower(0.1);
            try {
                Thread.sleep(50);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.leftWheel.setPower(-0.2);
            robot.rightWheel.setPower(0.2);
            try {
                Thread.sleep(50);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.leftWheel.setPower(-0.3);
            robot.rightWheel.setPower(0.3);
            try {
                Thread.sleep(50);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.leftWheel.setPower(-0.4);
            robot.rightWheel.setPower(0.4);
            try {
                Thread.sleep(50);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.leftWheel.setPower(-0.5);
            robot.rightWheel.setPower(0.5);
            try {
                Thread.sleep(50);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.leftWheel.setPower(-0.6);
            robot.rightWheel.setPower(0.6);
            try {
                Thread.sleep(50);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.leftWheel.setPower(-0.7);
            robot.rightWheel.setPower(0.7);
            try {
                Thread.sleep(500);
            } catch (java.lang.InterruptedException e) {
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
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.piston.setPower(1);
            try {
                Thread.sleep(1800);
            } catch (java.lang.InterruptedException e) {
                telemetry.addData("error", e);
            }
            robot.piston.setPower(0);
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);


        }
        robot.rightWheel.setPower(-variablePower);
        robot.leftWheel.setPower(variablePower);

        // Add telemetry data
        telemetry.addData("left",  "%.2f", leftScaled);
        telemetry.addData("right", "%.2f", rightScaled);
        telemetry.addData("", "x");
        telemetry.addData("", "y");
        telemetry.addData("", "a");
        telemetry.addData("", "b");
        updateTelemetry(telemetry);

    }

    double scaleInput(double dVal, double lt, double rt) {

        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
           dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        double more_scale = 1;
//        if (rt >= 0.1) {
//            more_scale = Range.clip(more_scale + 0.6 * rt, 0, 1);
//        } else
        if (lt >= 0.1) {
            more_scale = Range.clip(more_scale - 1.0 * lt, 0.05, 1);
        }
        dScale = dScale * more_scale;
        // return scaled value.
        return dScale;
    }


    @Override
    public void stop() {
    }

}
