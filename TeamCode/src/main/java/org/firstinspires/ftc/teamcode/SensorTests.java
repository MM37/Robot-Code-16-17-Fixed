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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Sensor Tests", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class SensorTests extends OpMode
{

    Hardware robot = new Hardware();
    ElapsedTime runtime = new ElapsedTime();

    int redValue = 0;
    int blueValue = 0;
    int greenValue = 0;
    int hue = 0;
    int light = 0;
    String connectionInfo = "";

    double lightDetected = 0;

    boolean pressed = false;



    public double findMax(double... vals) {
        double max = 0;

        for (double d : vals) {
            max = d > max ? d : max;
        }

        return max;
    }

    public void move(double power, double angle) {
        double y = Math.sin(Math.toRadians(angle));
        double x = Math.cos(Math.toRadians(angle));

        double setFL = y + x;
        double setFR = -y + x;
        double setBL = y - x;
        double setBR = -y - x;

        double max = findMax(Math.abs(setFL), Math.abs(setFR), Math.abs(setBL), Math.abs(setBR));

        double scale = power/max;

        setFL *= scale;
        setFR *= scale;
        setBL *= scale;
        setBR *= scale;

        runtime.reset();

        //telemetry.addData("Power", " BR: " + setBR + " BL: " + setBL + " FL: " + setFL + " FR: " + setFR);
        //telemetry.update();

        robot.BR.setPower(setBR);
        robot.FL.setPower(setFL);
        robot.FR.setPower(setFR);
        robot.BL.setPower(setBL);
    }

    public void rotate (double power, int direction) {
        //1 for clockwise, -1 for counter-clockwise THIS NEEDS TO BE CHECKED
        robot.BR.setPower(power * direction);
        robot.FL.setPower(power * direction);
        robot.FR.setPower(power * direction);
        robot.BL.setPower(power * direction);
    }

    public void stopColor(int color) {
        //1: red, 2: middle, 3: blue, 4: either, 5: both

        switch (color) {
            case 1:
                while(robot.colorSensor.red() < 4) {
                    telemetry.addData("Sensor Reading", robot.colorSensor.red());
                }
                break;
            case 2:
                while(robot.colorSensor.blue() < 2);
                break;
            case 3:
                while(robot.colorSensor.blue() < 2);
                break;
            case 4:
                while(robot.colorSensor.blue() == 0 && robot.colorSensor.red() == 0);
                break;
            case 5:
                while(robot.colorSensor.blue() == 0 || robot.colorSensor.red() == 0);
                break;
        }

        finalStop();
    }

    public void stopTime(double time) {
        runtime.reset();
        while(runtime.seconds() < time);
        finalStop();
    }

    public void stopTouch() {
        while(!robot.touchSensor.isPressed());
        finalStop();
    }

    public void finalStop() {
        robot.BR.setPower(0);
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);

        runtime.reset();
        while(runtime.seconds() < 1);
    }

    public void pressBeacon() {
        stopColor(1);
        telemetry.addData("b", runtime.seconds());
        telemetry.update();
        /*if(robot.colorSensor.red() <  1) {
            move(0.2, 180);
            stopColor(1);
        }*/
        move(0.15, 270);
        stopTouch();
        move(0.15, 180);
        stopTime(0.4);
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        robot.colorSensor.enableLed(true);
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
    public void start() {
        runtime.reset();
        //move(0.4, 225);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.lift.setPosition(-gamepad1.left_stick_y);

        redValue = robot.colorSensor.red();
        blueValue = robot.colorSensor.blue();
        greenValue = robot.colorSensor.green();
        hue = robot.colorSensor.argb();
        light = robot.colorSensor.alpha();
        connectionInfo = robot.colorSensor.getConnectionInfo();

        lightDetected = robot.distanceSensor.getLightDetected();

        pressed = robot.touchSensor.isPressed();

        telemetry.addData("Red:", redValue);
        telemetry.addData("Blue:", blueValue);
        telemetry.addData("Green:", greenValue);
        telemetry.addData("Hue:", hue);
        telemetry.addData("Light:", light);
        telemetry.addData("Connection Info:", connectionInfo);

        telemetry.addData("", "");

        telemetry.addData("Optical Light:", lightDetected);

        telemetry.addData("", "");

        telemetry.addData("Touch Sensor:", pressed);

        telemetry.addData("", "");

        telemetry.addData("Servo Min & Max Positions", robot.lift.MIN_POSITION + " " + robot.lift.MAX_POSITION);
        telemetry.addData("Servo Last Sent Position", robot.lift.getPosition());

        telemetry.addData("", "");

        telemetry.addData("Counter:", runtime.seconds());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

}
