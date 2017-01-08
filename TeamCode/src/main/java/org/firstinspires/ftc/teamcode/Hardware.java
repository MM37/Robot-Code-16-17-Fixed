package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Hardware
{
    Class base;
    HardwareMap hwMap  = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Drive Train
    public DcMotor  FL   = null;
    public DcMotor  FR  = null;
    public DcMotor  BL  = null;
    public DcMotor  BR  = null;

    //Other Motors
    public DcMotor lift = null;
    public DcMotor popper = null;
    public DcMotor pulley = null;

    //Other Servos
    public Servo ballPusher = null;

    //Sensors
    ColorSensor colorSensor = null;
    OpticalDistanceSensor distanceSensor = null;
    TouchSensor touchSensor = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
    //public void init(HardwareMap ahwMap, String aBase) {
        // save reference to HW Map
        try {
            //base = Class.forName(aBase);
        } catch (Exception e) {

        }
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.dcMotor.get("FL");
        FR = hwMap.dcMotor.get("FR");
        BL = hwMap.dcMotor.get("BL");
        BR = hwMap.dcMotor.get("BR");

        ballPusher = hwMap.servo.get("ballpusher");
        lift = hwMap.dcMotor.get("lift");
        popper = hwMap.dcMotor.get("popper");
        pulley = hwMap.dcMotor.get("pulley");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and Initialize Color Sensor
        colorSensor = hwMap.colorSensor.get("colorsensor");
        distanceSensor = hwMap.opticalDistanceSensor.get("distancesensor");
        touchSensor = hwMap.touchSensor.get("touchsensor");
    }

    final double whiteOpticalValue = 0.0025;

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

        BR.setPower(setBR);
        FL.setPower(setFL);
        FR.setPower(setFR);
        BL.setPower(setBL);
    }

    public void rotate (double power, int direction) {
        //1 for clockwise, -1 for counter-clockwise THIS NEEDS TO BE CHECKED
        BR.setPower(power * direction);
        FL.setPower(power * direction);
        FR.setPower(power * direction);
        BL.setPower(power * direction);
    }

    public void stopColor(int color) {
        //1: red, 2: middle, 3: blue, 4: either, 5: both

        switch (color) {
            case 1:
                while(colorSensor.red() < 4) {
                    //telemetry.addData("Sensor Reading", robot.colorSensor.red());
                }
                break;
            case 2:
                while(colorSensor.blue() < 2);
                break;
            case 3:
                while(colorSensor.blue() < 2);
                break;
            case 4:
                while(colorSensor.blue() == 0 && colorSensor.red() == 0);
                break;
            case 5:
                while(colorSensor.blue() == 0 || colorSensor.red() == 0);
                break;
        }

        finalStop();
    }

    public void stopTime(double time) {
        runtime.reset();
        try {
            //while(base.opModeIsActive() && runtime.seconds() < time);
        } catch (Exception e) {

        }
        finalStop();
    }

    public void stopTouch() {
        while(!touchSensor.isPressed());
        finalStop();
    }

    //stop either when it touches or time runs out
    public void stopTouchOrTime(double time) {
        runtime.reset();
        while(!touchSensor.isPressed() &&  runtime.seconds() < time);
        finalStop();
        if(!touchSensor.isPressed()) {
            rotate(0.13, 1);
        }
        finalStop();
    }

    public void stopWhite() {
        while(distanceSensor.getLightDetected() < whiteOpticalValue);
        finalStop();
    }

    //stops at white after ignoring for a certain time period
    public void stopWhiteAfterTime(double time) {
        runtime.reset();
        while(runtime.seconds() < time);
        while(distanceSensor.getLightDetected() < whiteOpticalValue);
        finalStop();
    }

    public void finalStop() {
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);

        runtime.reset();
        while(runtime.seconds() < 0.3);
    }

    //presses the beacon and backs up
    public void beaconPressRed() {
        if(colorSensor.blue() > colorSensor.red()) {
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
        if(colorSensor.red() > colorSensor.blue()) {
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

    /* Constructor */
    public Hardware() {
    }
}
