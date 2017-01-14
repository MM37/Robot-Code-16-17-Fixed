package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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
    public DcMotor  BL  = null;
    public DcMotor  FR  = null;
    public DcMotor  BR  = null;

    ModernRoboticsI2cGyro gyro = null;

    public final int wheelDiameter = 4;
    public final int pulsesPerRevolution = 1120;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.dcMotor.get("FL");
        BL = hwMap.dcMotor.get("BL");
        FR = hwMap.dcMotor.get("FR");
        BR = hwMap.dcMotor.get("BR");

        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");;

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move (double speed, double distance) {
        double revolutions = distance/(Math.PI * wheelDiameter);
        int ticks = (int) revolutions * pulsesPerRevolution;

        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);

        FL.setTargetPosition(ticks + FL.getCurrentPosition());
        BL.setTargetPosition(ticks + BL.getCurrentPosition());
        FR.setTargetPosition(ticks + FR.getCurrentPosition());
        BR.setTargetPosition(ticks + BR.getCurrentPosition());
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

        while (gyro.getHeading() != newOrientation); //need to add new op modes

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
}