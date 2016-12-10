package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Straight Run", group="Main Robot")
public class StraightRun extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //robot.move(0.4, 225);
        //robot.stopTime(10);

        robot.FL.setPower(0.75);
        robot.FR.setPower(-0.75);
        robot.BL.setPower(0.75);
        robot.BR.setPower(-0.75);

        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 8)) {

        }
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
    }
}
