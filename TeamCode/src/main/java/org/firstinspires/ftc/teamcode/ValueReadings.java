package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Value Reading", group="Testing")  // @Autonomous(...) is the other common choice
public class ValueReadings extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.odsLeft.enableLed(true);
        robot.odsRight.enableLed(true);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("ODS Left", robot.odsLeft.getLightDetected());
        telemetry.addData("ODS Left", robot.odsLeft.getRawLightDetected());
        telemetry.addData("ODS Right", robot.odsRight.getLightDetected());
        telemetry.addData("ODS Right", robot.odsRight.getRawLightDetected());
        telemetry.addData("Color Red", robot.colorSensor.red());
        telemetry.addData("Color Blue", robot.colorSensor.blue());
        telemetry.addData("Color Alpha", robot.colorSensor.alpha());
        telemetry.addData("Color Hue", robot.colorSensor.argb());
    }
}