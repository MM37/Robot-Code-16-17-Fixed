package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

@TeleOp(name="Main", group="Competition")  // @Autonomous(...) is the other common choice
public class TeleOpTankDrive extends OpMode {
    Hardware robot = new Hardware();

    double left;
    double right;
    boolean slow;

    double slides;
    boolean collectionIn;
    boolean collectionOut;

    boolean flipped = false;
    boolean recentlyPressed = false;
    boolean isInSlideAuto = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.ballRelease.setPosition(robot.ballReleaseUpPosition);
        robot.clamp.setPosition(robot.clampDownPosition);
        robot.liftRelease.setPosition(robot.liftReleaseClosePosition);
        robot.colorSensor.enableLed(false);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        //movement functions (gamepad1)
        slow = gamepad1.left_bumper;
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        if(slow) {
            left *= 0.5;
            right *= 0.5;
        }
        //allows driver to flip movement controls
        if (gamepad1.a && !recentlyPressed) {
            recentlyPressed = true;
            flipped = !flipped;
        } else if (!gamepad1.a && recentlyPressed) {
            recentlyPressed = false;
        }

        //collection movement
        collectionIn = gamepad1.right_bumper;
        collectionOut = gamepad1.right_trigger > 0.1;

        //other functions (gamepad2)
        slides = -gamepad2.left_stick_y;

        //ball release
        if (gamepad2.y) {
            robot.ballRelease.setPosition(robot.ballReleaseUpPosition);
        } else if (gamepad2.a) {
            robot.ballRelease.setPosition(robot.ballReleaseDownPosition);
        }
        //popper movement
        if (gamepad2.right_bumper) {
            //robot.popper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.popper.setPower(robot.popperSpeed);
        } else if (gamepad2.left_bumper) {
            //robot.popper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.popper.setPower(-robot.popperSpeed);
        } else {
            robot.popper.setPower(0);
        }

        //shoot thread
        if (gamepad2.right_trigger > 0.1) {
            Runnable shoot = new ShootCycle("shootTest");
            Thread shoot2 = new Thread(shoot);
            shoot2.start();
        }

        //lift thread
        if (gamepad2.left_trigger > 0.1) {
            Runnable lift = new CAPLift("litTest"); //REMOVE
            Thread lift2 = new Thread(lift);
            lift2.start();
        }

        //beacon back up thread
        if (gamepad2.dpad_left) {
            Runnable beaconMove = new FindLine("findTest"); //REMOVE
            Thread beacon2 = new Thread(beaconMove);
            beacon2.start();
        }

        //beacon-pushing servo movement
        if (gamepad2.dpad_up) {
            robot.beaconPusher.setPower(1);
            telemetry.addData("Debug", "Beacon Pusher Out");
        } else if (gamepad2.dpad_down) {
            robot.beaconPusher.setPower(-1);
            telemetry.addData("Debug", "Beacon Pusher In");
        } else {
            robot.beaconPusher.setPower(0);
        }

        //CAP ball clamp positioning
        if (gamepad2.b) {
            robot.clamp.setPosition(robot.clampUpPosition);
            robot.liftRelease.setPosition(robot.liftReleaseOpenPosition);
        }
        if (gamepad2.x) {
            robot.clamp.setPosition(robot.clampMediumPosition);
        }

        ///ensures that any power passed to motors isn't so low the motor burns out
        left = Math.abs(left) > 0.1 ? left : 0;
        right = Math.abs(right) > 0.1 ? right : 0;
        slides = Math.abs(slides) > 0.1 ? slides : 0;

        //drive train movement
        if (flipped) {
            robot.FL.setPower(-right);
            robot.BL.setPower(-right);
            robot.FR.setPower(-left);
            robot.BR.setPower(-left);
        } else {
            robot.FL.setPower(left);
            robot.BL.setPower(left);
            robot.FR.setPower(right);
            robot.BR.setPower(right);
        }

        //popper, slides, and collection movements
        /*if (slides > 0.1 && isInSlideAuto) {
            robot.slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.setPower(slides);
        } else if (!isInSlideAuto) {
            robot.slides.setPower(slides);
        }*/
        robot.slides.setPower(slides);

        if (collectionIn) {
            robot.collection.setPower(-robot.collectionSpeed);
        } else if (collectionOut) {
            robot.collection.setPower(robot.collectionSpeed);
        } else {
            robot.collection.setPower(0);
        }

        //ADD TELEMETRY
        telemetry.addData("Left ODS", robot.odsLeft);
        telemetry.addData("Right ODS", robot.odsRight);
    }

    class ShootCycle implements Runnable {
        private Thread t;
        private String threadName;

        ShootCycle(String name) {
            threadName = name;
        }

        public void run() {
            try {
                robot.ballRelease.setPosition(robot.ballReleaseDownPosition);
                sleep(1000);

                robot.ballRelease.setPosition(robot.ballReleaseUpPosition);
                sleep(100);

                robot.popper.setPower(robot.popperSpeed);
            } catch (InterruptedException e) {
                telemetry.addData("Thread Error", e.toString());
            }

        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }

    class CAPLift implements Runnable {
        private Thread t;
        private String threadName;

        CAPLift(String name) {
            threadName = name;
        }

        public void run() {
            /*isInSlideAuto = true;
            robot.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slides.setTargetPosition(robot.slides.getCurrentPosition() + robot.liftTicks);*/
            try {
                robot.slides.setPower(1);
                sleep(2000);
                robot.slides.setPower(0);
            } catch (InterruptedException e) {
                telemetry.addData("Thread Error", e.toString());
            }
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }

    class FindLine implements Runnable {
        private Thread t;
        private String threadName;

        FindLine(String name) {
            threadName = name;
        }

        public void run() {

            robot.FR.setPower(-0.15);
            robot.BR.setPower(-0.15);
            robot.FL.setPower(-0.15);
            robot.BL.setPower(-0.15);

            while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw && robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw && !gamepad2.dpad_right);

            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.FL.setPower(0);
            robot.BL.setPower(0);

            if (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw && !gamepad2.dpad_right) {
                robot.FR.setPower(0);
                robot.BR.setPower(0);
                while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw && !gamepad2.dpad_right);
                robot.FL.setPower(0);
                robot.BL.setPower(0);
            } else if (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw && !gamepad2.dpad_right) {
                robot.FL.setPower(0);
                robot.BL.setPower(0);
                while (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw && !gamepad2.dpad_right);
                robot.FR.setPower(0);
                robot.BR.setPower(0);
            }
        }

        public void start () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }
}