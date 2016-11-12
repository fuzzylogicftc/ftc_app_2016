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

@TeleOp(name="Bacon Drivetrain")
public class BaconDrivetrain extends OpMode{

    /* Declare OpMode members. */
    BaconBot robot = new BaconBot();
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
        double lt;
        double rt;
        double baconSpeed;
        boolean x;
        boolean y;
        double leftRaw;
        double rightRaw;
        double leftScaled;
        double rightScaled;
        double baconOffset = 0;

        lt = gamepad1.left_trigger;
        rt = gamepad1.right_trigger;
        x = gamepad1.x;
        y = gamepad1.y;

        leftRaw = -gamepad1.left_stick_y;
        rightRaw = -gamepad1.right_stick_y;

        leftScaled = scaleInput(leftRaw, lt, rt);
        rightScaled = scaleInput(rightRaw, lt, rt);

        // Use gamepad left & right Bumpers to open and close the claw
        if (x)
            baconOffset += robot.BACON_SPEED ;
        else if (y)
            baconOffset -= robot.BACON_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        baconOffset = Range.clip(baconOffset, -0.5, 0.5);
        robot.baconMotor.setPosition(robot.baconMotor.getPosition() + baconOffset);

        robot.leftMotor.setPower(leftScaled);
        robot.rightMotor.setPower(rightScaled);


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", leftScaled);
        telemetry.addData("right", "%.2f", rightScaled);
        telemetry.addData("gamepad",  gamepad1.toString());
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
        double more_scale = 0.4;
        if (rt >= 0.1) {
            more_scale = Range.clip(more_scale + 0.6 * rt, 0, 1);
        } else if (lt >= 0.1) {
            more_scale = Range.clip(more_scale - 0.4 * lt, 0.05, 1);
        }
        dScale = dScale * more_scale;
        // return scaled value.
        return dScale;
    }

    double getBacon(boolean rb, boolean lb) {
        double bacon = 0.0;
        if (rb) {
            bacon = robot.BACON_SPEED;
        }
        else if (lb) {
            bacon = -robot.BACON_SPEED;
        }
        return bacon;
    }

    @Override
    public void stop() {
    }

}
