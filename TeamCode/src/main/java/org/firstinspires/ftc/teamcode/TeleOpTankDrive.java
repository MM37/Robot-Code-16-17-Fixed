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

@TeleOp(name="teleop tank drive", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOpTankDrive extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //hardware variables
    Hardware robot           = new Hardware();
    boolean tankDrive = true;
    boolean slow;
    boolean wasPressed = false;


    double leftX = 0;
    double leftY = 0;
    double rightX = 0;
    double rightY = 0;

    double setFL = 0;
    double setFR = 0;
    double setBL = 0;
    double setBR = 0;

    double power = 0;
    double xVector = 0;
    double yVector = 0;
    double max = 0;
    double scale = 0;

    boolean popper;
    boolean lift;
    double pulley = 0;


    final double servoPushed = 0;
    final double servoUnpushed = 0.4;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
    }



    @Override
    public void loop() {
        //map values to gamepad
        leftX = gamepad1.left_stick_x;
        leftY = -gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        rightY = -gamepad1.right_stick_y;

        if(gamepad1.right_bumper && !wasPressed) {
            wasPressed = true;
            if(tankDrive) {
                tankDrive = false;
            } else {
                tankDrive = true;
            }
        } else {
            wasPressed = false;
        }

        slow = gamepad1.left_bumper;
        power = gamepad1.right_trigger;

        popper = gamepad2.a;
        lift = gamepad2.right_bumper;
        pulley = gamepad2.left_stick_y;

        //assigns values to the motor sets based off of tank drive mode
        if(tankDrive) {
            setFL = leftY;
            setBL = leftY;
            setFR = -rightY;
            setBR = -rightY;

            if (slow) {
                setFL *= 0.2;
                setBL *= 0.2;
                setFR *= 0.2;
                setBR *= 0.2;
            }
        } else {
            xVector = leftX/(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftX, 2)));
            yVector = leftY/(Math.sqrt(Math.pow(leftY, 2) + Math.pow(leftY, 2)));

            if (Double.isNaN(yVector)) {
                yVector = 0;
            }

            if (Double.isNaN(xVector)) {
                xVector = 0;
            }

            setFL = yVector + xVector + rightX;
            setFR = -yVector + xVector + rightX;
            setBL = yVector - xVector + rightX;
            setBR = -yVector - xVector + rightX;

            max = robot.findMax(Math.abs(setFL), Math.abs(setFR), Math.abs(setBL), Math.abs(setBR));
            scale = power/max;

            if (!Double.isNaN(scale) && !Double.isInfinite(scale)) {
                setFL *= scale;
                setFR *= scale;
                setBL *= scale;
                setBR *= scale;
            }
        }

        ///ensures that any power passed to motors isn't so low the motor burns out
        setFL = Math.abs(setFL) > 0.1 ? setFL : 0;
        setFR = Math.abs(setFR) > 0.1 ? setFR : 0;
        setBL = Math.abs(setBL) > 0.1 ? setBL : 0;
        setBR = Math.abs(setBR) > 0.1 ? setBR : 0;

        robot.FL.setPower(setFL);
        robot.FR.setPower(setFR);
        robot.BL.setPower(setBL);
        robot.BR.setPower(setBR);

        robot.pulley.setPower(pulley);
        if(lift) {
            robot.lift.setPower(0.8);
        } else {
            robot.lift.setPower(0);
        }
        if(popper) {
            robot.popper.setPower(-1);
        } else {
            robot.popper.setPower(0);
        }

        telemetry.addData("was pressed Status", wasPressed);
        telemetry.addData("move status", tankDrive);
    }
}