package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware
{

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
        while(runtime.seconds() < time);
        finalStop();
    }

    public void stopTouch() {
        while(!touchSensor.isPressed());
        finalStop();
    }

    public void finalStop() {
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);

        runtime.reset();
        while(runtime.seconds() < 1);
    }

    public void pressBeacon() {
        stopColor(1);
        //telemetry.addData("b", runtime.seconds());
        //telemetry.update();
        /*if(robot.colorSensor.red() <  1) {
            move(0.2, 180);
            stopColor(1);
        }*/
        move(0.15, 270);
        stopTouch();
        move(0.15, 180);
        stopTime(0.4);
    }

    private ElapsedTime runtime = new ElapsedTime();

    /* Public OpMode members. */
    public DcMotor  FL   = null;
    public DcMotor  FR  = null;
    public DcMotor  BL  = null;
    public DcMotor  BR  = null;

    public DcMotor pulley = null;
    public Servo ballPusher = null;
    public DcMotor popper = null;
    public Servo lift = null;

    ColorSensor colorSensor = null;
    OpticalDistanceSensor distanceSensor = null;
    TouchSensor touchSensor = null;

    double whiteThreshold;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Constructor */
    public Hardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.dcMotor.get("FL");
        FR = hwMap.dcMotor.get("FR");
        BL = hwMap.dcMotor.get("BL");
        BR = hwMap.dcMotor.get("BR");

        pulley = hwMap.dcMotor.get("pulley");
        popper = hwMap.dcMotor.get("popper");
        ballPusher = hwMap.servo.get("ballpusher");
        lift = hwMap.servo.get("lift");

        /*
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        */

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Color Sensor
        colorSensor = hwMap.colorSensor.get("colorsensor");
        distanceSensor = hwMap.opticalDistanceSensor.get("distancesensor");
        touchSensor = hwMap.touchSensor.get("touchsensor");
    }
}
