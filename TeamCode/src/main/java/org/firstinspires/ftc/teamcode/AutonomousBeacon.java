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

@Autonomous(name="beacon blue", group="Main Robot")
//@Disabled
public class AutonomousBeacon extends LinearOpMode {

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.move(0.4, 225);
        robot.pressBeacon();
        robot.move(0.2, 180);
        robot.pressBeacon();
        /*
        move(02, 135);
        stopTime(1);
        rotate(30, 1);
        pressBeacon();
        move(0.2, 180);
        pressBeacon();
        */

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
