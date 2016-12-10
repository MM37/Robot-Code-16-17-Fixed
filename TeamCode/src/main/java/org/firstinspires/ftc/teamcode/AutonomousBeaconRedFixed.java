package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Beacon Red Fixed", group="Main Robot")
public class AutonomousBeaconRedFixed extends LinearOpMode {

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

    public void stopTime(double time) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time);
        finalStop();
    }

    public void stopTouch() {
        while(opModeIsActive() && !robot.touchSensor.isPressed());
        finalStop();
    }

    //stop either when it touches or time runs out
    public void stopTouchOrTime(double time) {
        runtime.reset();
        while(opModeIsActive() && !robot.touchSensor.isPressed() &&  runtime.seconds() < time);
        finalStop();
        if(!robot.touchSensor.isPressed()) {
            rotate(0.13, 1);
        }
        finalStop();
    }

    public void stopWhite() {
        while(opModeIsActive() && robot.distanceSensor.getLightDetected() < robot.whiteOpticalValue);
        finalStop();
    }

    //stops at white after ignoring for a certain time period
    public void stopWhiteAfterTime(double time) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time && robot.distanceSensor.getLightDetected() < robot.whiteOpticalValue);
        finalStop();
    }

    public void finalStop() {
        robot.BR.setPower(0);
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.3);
    }

    //presses the beacon and backs up
    public void beaconPressRed() {
        if(robot.colorSensor.blue() > robot.colorSensor.red()) {
            //left
            move(0.2, 90);
            stopTime(0.18);
        } else {
            //right
            move(0.2, 270);
            stopTime(0.23);
        }
        move(0.2, 360);
        stopTouchOrTime(1.5);

        move(0.2, 180);
        stopTime(0.6);
    }

    //presses the beacon and backs up
    public void beaconPressBlue() {
        if(robot.colorSensor.red() > robot.colorSensor.blue()) {
            //left
            move(0.2, 90);
            stopTime(0.18);
        } else {
            //right
            move(0.2, 270);
            stopTime(0.23);
        }
        move(0.2, 360);
        stopTouchOrTime(1.5);

        move(0.2, 180);
        stopTime(0.6);
    }

    //hits the first beacon, moves, and hits the second
    public void beaconMethodRed() {
        beaconPressRed();

        move(0.25, 270);
        stopWhiteAfterTime(1);

        beaconPressRed();
    }

    //hits the first beacon, moves, and hits the second
    public void beaconMethodBlue() {
        beaconPressBlue();

        move(0.25, 90);
        stopWhiteAfterTime(1);

        beaconPressBlue();
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        move(0.5, 270);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.75);
        move(0.35, 270);
        stopWhite();

        rotate(0.15, 1);
        stopTime(1.2);

        move(0.15, 90);
        stopWhite();

        beaconMethodRed();

        move(0.3, 270);
        stopTime(0.9);

        //90 degree corner rotate
        rotate(0.2, 1);
        stopTime(1.7);

        move(0.3, 270);
        stopWhite();

        beaconMethodRed();
    }
}