package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Hardware
{
    HardwareMap hwMap  = null;

    //Drive Train
    public DcMotor  FL   = null;
    public DcMotor  BL  = null;
    public DcMotor  FR  = null;
    public DcMotor  BR  = null;

    public DcMotor popper = null;
    public DcMotor collection = null;
    public DcMotor slides =  null;

    CRServo beaconPusher = null;
    Servo clamp = null;
    Servo liftRelease = null;
    Servo ballRelease = null;

    ModernRoboticsI2cGyro gyro = null;
    OpticalDistanceSensor odsLeft = null;
    OpticalDistanceSensor odsRight = null;
    ColorSensor colorSensor = null;

    public final int wheelDiameter = 4;
    public final int pulsesPerRevolution = 1120;
    public final double collectionSpeed = -1;
    public final double popperSpeed = 0.4;
    public final double beaconPressPosition = 0;
    public final double beaconReleasePosition = 1;
    public final double ballReleaseDownPosition = 0.74;
    public final double ballReleaseUpPosition = 0.14;
    public final double clampUpPosition = 0;
    public final double clampMediumPosition = 0.25;
    public final double clampDownPosition = 0.88;
    public final double liftReleaseOpenPosition = 0.5;
    public final double liftReleaseClosePosition = 0.9;
    public final double leftODSWhiteRaw = 0.15;
    public final double rightODSWhiteRaw = 0.2;
    public final int liftTicks = pulsesPerRevolution * 2;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.dcMotor.get("FL");
        BL = hwMap.dcMotor.get("BL");
        FR = hwMap.dcMotor.get("FR");
        BR = hwMap.dcMotor.get("BR");

        popper = hwMap.dcMotor.get("popper");
        collection = hwMap.dcMotor.get("collection");
        slides = hwMap.dcMotor.get("slides");

        beaconPusher = hwMap.crservo.get("beaconpusher");
        clamp = hwMap.servo.get("clamp");
        ballRelease = hwMap.servo.get("ballrelease");
        liftRelease = hwMap.servo.get("liftrelease");

        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        odsLeft = hwMap.opticalDistanceSensor.get("odsleft");
        odsRight = hwMap.opticalDistanceSensor.get("odsright");
        colorSensor = hwMap.colorSensor.get("colorsensor");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move (double speed, double distance) {
        int ticks = (int) ((pulsesPerRevolution * distance)/(Math.PI * wheelDiameter));

        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);

        FL.setTargetPosition(FL.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks);
        FR.setTargetPosition(FR.getCurrentPosition() + ticks);
        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
    }

    public void ballReleaseCycle () {
        ballRelease.setPosition(ballReleaseDownPosition);
        //sleep(1000);

        ballRelease.setPosition(ballReleaseUpPosition);
        //sleep(2500);
    }

    public void shootParticle() {
        popper.setTargetPosition(popper.getCurrentPosition() + pulsesPerRevolution);
        //while (popper.isBusy());
    }

    public void rotate (double speed, int degrees) {
        int newOrientation = gyro.getHeading() + degrees;
        if (newOrientation > 359) {
            newOrientation -= 360;
        } else if (newOrientation < 0) {
            newOrientation += 360;
        }

        if(degrees > 0) {
            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(-speed);
            BR.setPower(-speed);
        } else if (degrees < 0) {
            FL.setPower(-speed);
            BL.setPower(-speed);
            FR.setPower(speed);
            BR.setPower(speed);
        }

        while (gyro.getHeading() != newOrientation);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
}