package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="encoder test", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class EncoderTest extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.FL.setPower(0.5);
        robot.FL.setTargetPosition(robot.FL.getCurrentPosition() + 11);
    }

    @Override
    public void loop() {

    }
}