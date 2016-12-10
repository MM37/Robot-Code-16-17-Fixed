package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Beacon Red Half", group="Main Robot")
public class AutonomousBeaconRedHalf extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.move(0.5, 270);
        runtime.reset();
        while (runtime.seconds() < 1.75);
        robot.move(0.35, 270);
        robot.stopWhite();

        robot.rotate(0.15, 1);
        robot.stopTime(1.2);

        robot.move(0.15, 90);
        robot.stopWhite();

        robot.beaconMethodRed();
    }
}