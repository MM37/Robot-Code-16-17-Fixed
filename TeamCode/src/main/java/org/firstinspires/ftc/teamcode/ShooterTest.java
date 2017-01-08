package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Shooter Tests", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class ShooterTest extends OpMode
{

    DcMotor shooter = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        shooter = hardwareMap.dcMotor.get("shooter");
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
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        shooter.setPower(1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}
