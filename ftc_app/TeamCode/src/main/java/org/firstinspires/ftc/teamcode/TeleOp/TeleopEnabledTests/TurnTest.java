package org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;

/**
 * Created by Joshua on 2/8/2018.
 */

public class TurnTest extends TeleopEnabledTest{

    int amount = 0;
    double previousTime;
    RobotHardware hw = RobotHardware.GetSingleton();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void run() {
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain();
        drivetrain.setRunningOpMode(teleop);
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!teleop.gamepad1.back) {
            boolean clockwise = false;

            while (!teleop.gamepad1.y) {
                double deltaTime = timer.milliseconds() - previousTime;
                teleop.telemetry.addData("Encoder target", amount);
                teleop.telemetry.addData("Clockwise", clockwise);
                teleop.telemetry.update();
                if (teleop.gamepad1.left_bumper) {
                    clockwise = true;
                }
                if (teleop.gamepad1.right_bumper) {
                    clockwise = false;
                }
                amount += (teleop.gamepad1.dpad_up) ? deltaTime * 100 : 0;
                amount += (teleop.gamepad1.dpad_down) ? deltaTime * -100 : 0;
                amount += (teleop.gamepad1.a) ? deltaTime * 10 : 0;
                amount += (teleop.gamepad1.b) ? deltaTime * -10 : 0;
                previousTime = timer.milliseconds();
            }

            drivetrain.EncoderTurn(0.15, amount, clockwise);
        }
    }
}
