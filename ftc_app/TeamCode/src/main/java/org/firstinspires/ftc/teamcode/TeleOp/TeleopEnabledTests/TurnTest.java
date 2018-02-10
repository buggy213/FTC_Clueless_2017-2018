package org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests;

import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;

/**
 * Created by Joshua on 2/8/2018.
 */

public class TurnTest extends TeleopEnabledTest{

    int amount = 0;

    @Override
    public void run() {
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain();
        drivetrain.setRunningOpMode(teleop);
        while (!teleop.gamepad1.back) {
            boolean clockwise = false;

            while (!teleop.gamepad1.x) {
                teleop.telemetry.addData("Encoder target", amount);
                teleop.telemetry.addData("Clockwise", clockwise);
                teleop.telemetry.update();
                if (teleop.gamepad1.left_bumper) {
                    clockwise = true;
                }
                if (teleop.gamepad1.right_bumper) {
                    clockwise = false;
                }
                amount += (teleop.gamepad1.dpad_up) ? 10 : 0;
                amount += (teleop.gamepad1.dpad_down) ? -10 : 0;
                amount += (teleop.gamepad1.a) ? 1 : 0;
                amount += (teleop.gamepad1.b) ? -1 : 0;

            }
 
            drivetrain.EncoderTurn(0.15, amount, false);
        }
    }
}
