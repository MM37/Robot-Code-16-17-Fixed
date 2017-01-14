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

    public final int wheelDiameter = 4;
    public final int pulsesPerRevolution = 1120;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.dcMotor.get("FL");
        FR = hwMap.dcMotor.get("FR");
        BL = hwMap.dcMotor.get("BL");
        BR = hwMap.dcMotor.get("BR");

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move (double speed, double distance) {
        double revolutions = distance/(Math.PI * wheelDiameter);
        int ticks = (int) revolutions * pulsesPerRevolution;

        FL.setPower(speed);
        FR.setPower(speed);
        BL.setPower(speed);
        BR.setPower(speed);

        FL.setTargetPosition(ticks + FL.getCurrentPosition());
        FR.setTargetPosition(ticks + FR.getCurrentPosition());
        BL.setTargetPosition(ticks + BL.getCurrentPosition());
        BR.setTargetPosition(ticks + BR.getCurrentPosition());
    }
}