package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Beacon Red", group="Main Robot")
public class AutonomousBeaconRed extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.FL.setPower(0.5);

        telemetry.addData("Status", "power set");
        telemetry.update();

        robot.FL.setTargetPosition(robot.FL.getCurrentPosition() + 3360);

        telemetry.addData("Status", "target set");
        telemetry.update();

        while (true) {
            telemetry.addData("Current", robot.FL.getCurrentPosition());
            telemetry.addData("Seconds", runtime.seconds());
            telemetry.update();
        }
    }
}