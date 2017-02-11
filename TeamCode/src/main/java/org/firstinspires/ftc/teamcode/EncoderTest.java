package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Encoder Test", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class EncoderTest extends OpMode {
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Position", robot.gyro.getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        if (robot.gyro.getHeading() > 180) {
            robot.FL.setPower(0.3);
            robot.BL.setPower(0.3);
            robot.FR.setPower(-0.3);
            robot.BR.setPower(-0.3);
            while (robot.gyro.getHeading() > 180);
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
        } else if (robot.gyro.getHeading() <= 180) {
            robot.FL.setPower(-0.3);
            robot.BL.setPower(-0.3);
            robot.FR.setPower(0.3);
            robot.BR.setPower(0.3);
            while (robot.gyro.getHeading() <= 180);
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
        }
    }

    @Override
    public void loop() {
        telemetry.addData("Current Position", robot.gyro.getHeading());
        /*if (gamepad1.a) {
            Runnable left = new MoveBackLeft("test");
            Thread test = new Thread(left);
            test.start();
        }

        if (gamepad1.b) {
            Runnable right = new MoveBackRight("test");
            Thread test2 = new Thread(right);
            test2.start();
        }*/
    }

    class MoveBackLeft implements Runnable {
        private Thread t;
        private String threadName;

        MoveBackLeft( String name) {
            threadName = name;
        }

        public void run() {
            try {
                robot.FL.setPower(-0.3);
                robot.BL.setPower(-0.3);
                Thread.sleep(2000);
                robot.FL.setPower(0);
                robot.BL.setPower(0);
            }catch (InterruptedException e) {
                telemetry.addData("Error", threadName);
            }
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }

    class MoveBackRight implements Runnable {
        private Thread t;
        private String threadName;

        MoveBackRight( String name) {
            threadName = name;
        }

        public void run() {
            try {
                robot.FR.setPower(-0.3);
                robot.BR.setPower(-0.3);
                Thread.sleep(2000);
                robot.FR.setPower(0);
                robot.BR.setPower(0);
            }catch (InterruptedException e) {
                telemetry.addData("Error", threadName);
            }
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }

    class MyThread implements Runnable {
        private Thread t;
        private String threadName;

        MyThread( String name) {
            threadName = name;
        }

        public void run() {
            //Thread Code Goes Here
            //You May Need a Try-Catch Block
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }
}
