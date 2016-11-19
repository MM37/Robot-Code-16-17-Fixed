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
    /* Public OpMode members. */
    public DcMotor  FL   = null;
    public DcMotor  FR  = null;
    public DcMotor  BL  = null;
    public DcMotor  BR  = null;

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
        FL   = hwMap.dcMotor.get("FL");
        FR   = hwMap.dcMotor.get("FR");
        BL   = hwMap.dcMotor.get("BL");
        BR   = hwMap.dcMotor.get("BR");

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
