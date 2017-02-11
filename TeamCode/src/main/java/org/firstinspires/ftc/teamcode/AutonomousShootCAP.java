package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

@Autonomous(name="Shoot & CAP Red", group="Competition")
public class AutonomousShootCAP extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.ballRelease.setPosition(robot.ballReleaseUpPosition);
        robot.clamp.setPosition(robot.clampDownPosition);
        robot.liftRelease.setPosition(robot.liftReleaseClosePosition);
        robot.colorSensor.enableLed(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.gyro.calibrate();

        while (robot.gyro.isCalibrating());

        waitForStart();

        move(0.25, -12);
        sleep(3000);

        robot.popper.setPower(robot.popperSpeed);
        robot.popper.setTargetPosition(robot.popper.getCurrentPosition() + robot.pulsesPerRevolution);
        sleep(2000);

        robot.ballRelease.setPosition(robot.ballReleaseDownPosition);
        sleep(1000);

        robot.ballRelease.setPosition(robot.ballReleaseUpPosition);
        sleep(2500);

        robot.popper.setTargetPosition(robot.popper.getCurrentPosition() + robot.pulsesPerRevolution);
        sleep(2000);

        robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate(0.3, -20);
        sleep(2000);

        move(0.25, -15);
        sleep(3000);

        /*rotate(0.3, 125);
        sleep(3000);

        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        moveUp(0.3, 66);
        sleep(5000);

        robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int temp = robot.gyro.getHeading();

        robot.BL.setPower(0.35);
        robot.FL.setPower(0.35);

        while (robot.gyro.getHeading() < temp + 38);

        robot.FL.setPower(0);
        robot.BL.setPower(0);

        sleep(500);

        telemetry.addData("Debug", "Left 1");
        telemetry.update();
        Runnable left = new MoveBackODSLeft("firstLeft");
        telemetry.addData("Debug", "Left 2");
        telemetry.update();
        Thread leftThread = new Thread(left);

        telemetry.addData("Debug", "Right 1");
        telemetry.update();
        Runnable right = new MoveBackODSLeft("firstRight");
        telemetry.addData("Debug", "Right 2");
        telemetry.update();
        Thread rightThread = new Thread(right);

        telemetry.addData("Debug", "Right 3");
        telemetry.update();
        rightThread.start();
        telemetry.addData("Debug", "Left 3");
        telemetry.update();
        //leftThread.start();

        telemetry.addData("Debug", "Done");
        telemetry.update();

        sleep(8000);

        robot.FR.setPower(-0.1);
        robot.BR.setPower(-0.1);
        robot.FL.setPower(-0.1);
        robot.BL.setPower(-0.1);

        while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw && robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw);

        robot.FR.setPower(0);
        robot.BR.setPower(0);
        robot.FL.setPower(0);
        robot.BL.setPower(0);

        if (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw) {
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw);
            robot.FL.setPower(0);
            robot.BL.setPower(0);

            /*robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.FR.setTargetPosition(1);
            robot.BR.setTargetPosition(1);
        } else if (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw) {
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            while (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw);
            robot.FR.setPower(0);
            robot.BR.setPower(0);

            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.FL.setTargetPosition(1);
            robot.BL.setTargetPosition(1);
        }

            if (robot.gyro.getHeading() > 180) {
                robot.FL.setPower(-0.15);
                robot.BL.setPower(-0.15);
                robot.FR.setPower(0.15);
                robot.BR.setPower(0.15);
                while (robot.gyro.getHeading() > 180);
                robot.FL.setPower(0);
                robot.BL.setPower(0);
                robot.FR.setPower(0);
                robot.BR.setPower(0);
            } else if (robot.gyro.getHeading() < 180) {
                robot.FL.setPower(0.15);
                robot.BL.setPower(0.15);
                robot.FR.setPower(-0.15);
                robot.BR.setPower(-0.15);
                while (robot.gyro.getHeading() < 180);
                robot.FL.setPower(0);
                robot.BL.setPower(0);
                robot.FR.setPower(0);
                robot.BR.setPower(0);
            }

        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveUp(0.3, 4);
        sleep(3000);

        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            telemetry.addData("Color", "Red");
            telemetry.update();
            moveUp(0.3, 2);
            sleep(3000);
        } else if (robot.colorSensor.blue() > robot.colorSensor.red()) {
            telemetry.addData("Color", "Red");
            telemetry.update();
            moveUp(0.3, 5);
            sleep(3000);
        } else {
            telemetry.addData("Color", "Error!");
            telemetry.update();
        }*/
    }

    public void move (double speed, double distance) {
        int original = robot.FL.getCurrentPosition();
        double revolutions = distance/(Math.PI * robot.wheelDiameter);
        //int ticks = (int) revolutions * pulsesPerRevolution;
        int ticks = (int) ((robot.pulsesPerRevolution * distance)/(Math.PI * robot.wheelDiameter));

        robot.FL.setPower(speed);
        robot.BL.setPower(speed);
        robot.FR.setPower(speed);
        robot.BR.setPower(speed);

        robot.FL.setTargetPosition(robot.FL.getCurrentPosition() + ticks);
        robot.BL.setTargetPosition(robot.BL.getCurrentPosition() + ticks);
        robot.FR.setTargetPosition(robot.FR.getCurrentPosition() + ticks);
        robot.BR.setTargetPosition(robot.BR.getCurrentPosition() + ticks);

        while (robot.FL.getCurrentPosition() > robot.FL.getTargetPosition()) {
            telemetry.addData("Original", original);
            telemetry.addData("Target", robot.FL.getTargetPosition());
            telemetry.addData("Ticks", ticks);
            telemetry.addData("Current Distance", robot.FL.getCurrentPosition());
            telemetry.update();
        }

        robot.FL.setPower(0);
        robot.BL.setPower(0);
        robot.FR.setPower(0);
        robot.BR.setPower(0);

    }

    public void moveUp (double speed, double distance) {
        int original = robot.FL.getCurrentPosition();
        double revolutions = distance/(Math.PI * robot.wheelDiameter);
        //int ticks = (int) revolutions * pulsesPerRevolution;
        int ticks = (int) ((robot.pulsesPerRevolution * distance)/(Math.PI * robot.wheelDiameter));

        robot.FL.setPower(speed);
        robot.BL.setPower(speed);
        robot.FR.setPower(speed);
        robot.BR.setPower(speed);

        robot.FL.setTargetPosition(robot.FL.getCurrentPosition() + ticks);
        robot.BL.setTargetPosition(robot.BL.getCurrentPosition() + ticks);
        robot.FR.setTargetPosition(robot.FR.getCurrentPosition() + ticks);
        robot.BR.setTargetPosition(robot.BR.getCurrentPosition() + ticks);

        while (robot.FL.getCurrentPosition() < robot.FL.getTargetPosition()) {
            telemetry.addData("Original", original);
            telemetry.addData("Target", robot.FL.getTargetPosition());
            telemetry.addData("Ticks", ticks);
            telemetry.addData("Current Distance", robot.FL.getCurrentPosition());
            telemetry.update();
        }

        robot.FL.setPower(0);
        robot.BL.setPower(0);
        robot.FR.setPower(0);
        robot.BR.setPower(0);

    }

    public void rotate (double speed, int degrees) {
        int original = robot.gyro.getHeading();
        int newOrientation = original + degrees;
        if (newOrientation > 359) {
            newOrientation -= 360;
        } else if (newOrientation < 0) {
            newOrientation += 360;
        }

        if(degrees > 0) {
            robot.FL.setPower(speed);
            robot.BL.setPower(speed);
            robot.FR.setPower(-speed);
            robot.BR.setPower(-speed);
            while (robot.gyro.getHeading() > newOrientation);
            while (robot.gyro.getHeading() < newOrientation);
        } else if (degrees < 0) {
            robot.FL.setPower(-speed);
            robot.BL.setPower(-speed);
            robot.FR.setPower(speed);
            robot.BR.setPower(speed);
            while (robot.gyro.getHeading() < newOrientation);
            while (robot.gyro.getHeading() > newOrientation);
        }

        robot.FL.setPower(0);
        robot.BL.setPower(0);
        robot.FR.setPower(0);
        robot.BR.setPower(0);
    }

    public void shootParticle() {
        robot.popper.setTargetPosition(robot.popper.getCurrentPosition() + robot.pulsesPerRevolution);
        //while (popper.isBusy());
    }

    class MoveBackODSLeft implements Runnable {
        private Thread t;
        private String threadName;

        MoveBackODSLeft( String name) {
            threadName = name;
        }

        public void run() {
            robot.FL.setPower(-0.2);
            robot.BL.setPower(-0.2);
            telemetry.addData("Debug Class", "Left Started");
            telemetry.update();
            while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw);
            telemetry.addData("Debug Class", "Left Triggered");
            telemetry.update();
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            telemetry.addData("Debug Class", "Left Ended");
            telemetry.update();
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }

    class MoveBackODSRight implements Runnable {
        private Thread t;
        private String threadName;

        MoveBackODSRight( String name) {
            threadName = name;
        }

        public void run() {
            robot.FR.setPower(-0.2);
            robot.BR.setPower(-0.2);
            telemetry.addData("Debug Class", "Right Started");
            telemetry.update();
            while (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw);
            telemetry.addData("Debug Class", "Right Triggered");
            telemetry.update();
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            telemetry.addData("Debug Class", "Right Ended");
            telemetry.update();
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }
}