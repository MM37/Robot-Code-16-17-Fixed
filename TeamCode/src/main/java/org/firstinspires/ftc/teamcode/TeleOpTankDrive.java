package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop tank drive", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class TeleOpTankDrive extends OpMode {
    Hardware robot = new Hardware();
    double left;
    double right;
    boolean slow;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        slow = gamepad1.left_bumper;
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        if(slow) {
            left *= 0.5;
            right *= 0.5;
        }

        ///ensures that any power passed to motors isn't so low the motor burns out
        left = Math.abs(left) > 0.1 ? left : 0;
        right = Math.abs(right) > 0.1 ? right : 0;

        robot.FL.setPower(left);
        robot.BL.setPower(left);
        robot.FR.setPower(right);
        robot.BR.setPower(right);
    }
}