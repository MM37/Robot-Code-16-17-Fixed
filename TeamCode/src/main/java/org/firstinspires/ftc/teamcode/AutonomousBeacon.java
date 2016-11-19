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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="beacon", group="Main Robot")
//@Disabled
public class AutonomousBeacon extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

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

    public void stopColor(int color) {
        //1: red, 2: middle, 3: blue, 4: either, 5: both

        switch (color) {
            case 1:
                while(robot.colorSensor.red() == 0);
                break;
            case 2:
                while(robot.colorSensor.blue() == 0);
                break;
            case 3:
                while(robot.colorSensor.blue() < 1);
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

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        move(0.3, 225);
        stopColor(3);
        move(0.25, 270);
        stopTouch();

        /*
        move(0.2, 90);
        stopTime(0.25);

        telemetry.addData("Part", "1");
        telemetry.update();

        if(robot.colorSensor.blue() > 0) {

            telemetry.addData("Part", "1.1");
            telemetry.update();

            move(0.2, 360);
            telemetry.addData("Part", "1.1.1");
            telemetry.update();
            while (robot.colorSensor.blue() > 0)
            telemetry.addData("Part", "1.1.2");
            telemetry.update();
            finalStop();

            telemetry.addData("Part", "1.2");
            telemetry.update();

            if (robot.colorSensor.red()>0) {
                //found color intersection
                move(0.2, 180);
                stopTime(0.4);
            } else {
                //found beacon edge
                move(0.2, 180);
                stopColor(1);
                move(0.2, 360);
                stopTime(0.4);
            }

            telemetry.addData("Part", "1.3");
            telemetry.update();
        }

        telemetry.addData("Part", "2");
        telemetry.update();

        if(robot.colorSensor.red() > 0) {
            telemetry.addData("Part", "2.1");
            telemetry.update();

            move(0.2, 360);
            while (robot.colorSensor.red() > 0)
            finalStop();

            telemetry.addData("Part", "2.2");
            telemetry.update();

            if (robot.colorSensor.blue()>0) {
                //found color intersection
                move(0.2, 360);
                stopTime(0.4);
            } else {
                //found beacon edge
                move(0.2, 180);
                stopColor(2);
                move(0.2, 180);
                stopTime(0.4);
            }

            telemetry.addData("Part", "2.3");
            telemetry.update();
        }*/

        telemetry.addData("Part", "2");
        telemetry.update();

        while(true){}
    }
}
