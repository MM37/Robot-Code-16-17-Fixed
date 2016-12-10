package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Sensor Tests", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class SensorTests extends OpMode
{

    Hardware robot = new Hardware();
    ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        robot.distanceSensor.enableLed(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        /*robot.shooter1.setPower(0.05);
        robot.shooter2.setPower(-0.05);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.1);
        robot.shooter2.setPower(-0.1);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.15);
        robot.shooter2.setPower(-0.15);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.2);
        robot.shooter2.setPower(-0.2);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.25);
        robot.shooter2.setPower(-0.25);*/
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //telemetry.addData("Optical Light:", robot.distanceSensor.getLightDetected());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("FR Mode", robot.FR.getMode());
        telemetry.addData("Max FR Speed", robot.FR.getMaxSpeed());
        telemetry.addData("FR Zero Behavior", robot.FR.getZeroPowerBehavior());
        telemetry.addData("FR Encoder Continuous", robot.FR.getCurrentPosition());
        telemetry.addData("BL Encoder Continuous", robot.FR.getCurrentPosition());
        telemetry.addData("light", robot.distanceSensor.getLightDetected());
        telemetry.addData("hey", "justin");

        //robot.pulley.setPower(-0.2);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        /*robot.shooter1.setPower(0.2);
        robot.shooter2.setPower(-0.2);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.15);
        robot.shooter2.setPower(-0.15);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.1);
        robot.shooter2.setPower(-0.1);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0.05);
        robot.shooter2.setPower(-0.05);

        runtime.reset();
        while (runtime.seconds()< 1);

        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);*/
    }
}
