package org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests;

import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;

/**
 * Created by hsunx on 2/20/2018.
 */

public class AutoGrabTest extends TeleopEnabledTest {
    RobotHardware robot = RobotHardware.GetSingleton();

    @Override
    public void init() {
        description = "Test to determine best method for grabbing glyphs during autonomous" +"\n"
                    + "Press back to return, press Y to start";
    }

    @Override
    public void run() throws InterruptedException{
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain();
        while (!teleop.gamepad1.back) {
            robot.altClawLeft.setPosition(0.29);
            robot.altClawRight.setPosition(0.65);
            robot.upperLeft.setPosition(0.19);
            robot.upperRight.setPosition(0.77);
            while (!teleop.gamepad1.y) {

            }
            drivetrain.AutoMove(0.2, 0, 500);
            robot.altClawLeft.setPosition(0.67);  //0.610
            robot.altClawRight.setPosition(0.30);  //0.276
            robot.upperLeft.setPosition(0.61);  //0.6
            robot.upperRight.setPosition(0.33);  //0.36
            Thread.sleep(1000);
            drivetrain.AutoMove(0.2, 180, 500);

        }
    }
}
