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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="yunglean", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOp6337 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //hardware variables
    Hardware robot           = new Hardware();
    double xJoy; //x-axis left joystick
    double yJoy; //y-axis left joystick
    double rJoy; //x-axis right joystick

    double x; //x-axis left joystick
    double y; //y-axis left joystick

    double maxUndividedPower = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
    }

    public double findMax(double... vals) {
        double max = 0;

        for (double d : vals) {
            max = d > max ? d : max;
        }

        return max;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        xJoy = gamepad1.left_stick_x;
        yJoy = -gamepad1.left_stick_y;
        rJoy = gamepad1.right_stick_x;

        telemetry.addData("raw x input from joystick", xJoy);
        telemetry.addData("raw y input from joystick", yJoy);
        telemetry.addData("raw r input from joystick", rJoy);

        telemetry.addData("", "");

        x = xJoy/(Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
        y = yJoy/(Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));

        if (Double.isNaN(y)) {
            y = 0;
        }

        if (Double.isNaN(x)) {
            x = 0;
        }

        //finds the distance from the center
        double power = gamepad1.right_trigger;
        telemetry.addData("Undivided power", power);

        telemetry.addData("", "");

        double setFL = y + x + rJoy;
        double setFR = -y + x + rJoy;
        double setBL = y - x + rJoy;
        double setBR = -y - x + rJoy;

        telemetry.addData("unscaled setFL", setFL);
        telemetry.addData("unscaled setFR", setFR);
        telemetry.addData("unscaled setBL", setBL);
        telemetry.addData("unscaled setBR", setBR);

        telemetry.addData("", "");

        double max = findMax(Math.abs(setFL), Math.abs(setFR), Math.abs(setBL), Math.abs(setBR));

        double scale = power/max;

        telemetry.addData("power", power);
        telemetry.addData("max", max);
        telemetry.addData("scale", scale);

        telemetry.addData("", "");

        if (!Double.isNaN(scale) && !Double.isInfinite(scale)) {
            setFL *= scale;
            setFR *= scale;
            setBL *= scale;
            setBR *= scale;
        }

        telemetry.addData("scaled setFL", setFL);
        telemetry.addData("scaled setFR", setFR);
        telemetry.addData("scaled setBL", setBL);
        telemetry.addData("scaled setBR", setBR);

        robot.FL.setPower(setFL);
        robot.FR.setPower(setFR);
        robot.BL.setPower(setBL);
        robot.BR.setPower(setBR);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

}

/*
x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;

        telemetry.addData("raw x input from joystick", x);
        telemetry.addData("raw y input from joystick", y);
        telemetry.addData("raw r input from joystick", r);


        telemetry.addData("", "");


        //finds the distance from the center
        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        if (power > maxUndividedPower) {
            maxUndividedPower = power;
        }
        telemetry.addData("Undivided power", power);
        telemetry.addData("Max Undivided power", maxUndividedPower);

        //scales down to 1 to normalize values
        power /= 1.165932903;
        if (!gamepad1.right_bumper) {
            power /= 2;
        }
        telemetry.addData("Divided power", power);

        telemetry.addData("", "");

        double setFL = y + x + r;
        double setFR = -y + x + r;
        double setBL = y - x + r;
        double setBR = -y - x + r;

        telemetry.addData("unscaled setFL", setFL);
        telemetry.addData("unscaled setFR", setFR);
        telemetry.addData("unscaled setBL", setBL);
        telemetry.addData("unscaled setBR", setBR);

        telemetry.addData("", "");

        double max = findMax(Math.abs(setFL), Math.abs(setFR), Math.abs(setBL), Math.abs(setBR));

        double scale = power/max;

        if (!Double.isNaN(scale)) {
            setFL *= scale;
            setFR *= scale;
            setBL *= scale;
            setBR *= scale;
        }

        telemetry.addData("scaled setFL", setFL);
        telemetry.addData("scaled setFR", setFR);
        telemetry.addData("scaled setBL", setBL);
        telemetry.addData("scaled setBR", setBR);

        robot.FL.setPower(setFL);
        robot.FR.setPower(setFR);
        robot.BL.setPower(setBL);
        robot.BR.setPower(setBR);
 */