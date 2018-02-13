package org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Shared.RobotHardware;

/**
 * Created by Joshua on 2/12/2018.
 */

public class ServosAndMotorsTest extends TeleopEnabledTest {

    RobotHardware hw = RobotHardware.GetSingleton();
    Gamepad previousGamepad1;

    boolean motors;

    int motorIndex = 0;
    int servoIndex = 0;

    double motorPower;
    double servoPosition;

    DcMotor currentMotor;
    Servo currentServo;

    public void updateCurrentMotor(int index) {
        try {
            currentMotor = (DcMotor) hw.dcMotors.get(index).get(hw);
            currentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e) {
            RobotLog.e(e.toString());
        }
    }

    public void updateCurrentServo(int index) {
        try {
            currentServo = (Servo) hw.servos.get(index).get(hw);
        }
        catch (Exception e) {
            RobotLog.e(e.toString());
        }
    }

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        description = "Servo and motor angle/power/encoding testing program";
    }

    @Override
    public void run() {

        updateCurrentMotor(0);
        updateCurrentServo(0);

        double previousTime = 0;
        timer.reset();
        while (!teleop.gamepad1.back) {

            double deltaTime = (timer.milliseconds() - previousTime) / 1000;
            if (teleop.gamepad1.x && !previousGamepad1.x) {
                motors = !motors;
            }

            if (motors) {
                if (teleop.gamepad1.left_bumper && !previousGamepad1.left_bumper && motorIndex > 0) {
                    motorIndex--;
                    updateCurrentMotor(motorIndex);
                }
                if (teleop.gamepad1.right_bumper && !previousGamepad1.right_bumper && motorIndex < (hw.dcMotors.size() - 1)) {
                    motorIndex++;
                    updateCurrentMotor(motorIndex);
                }

                motorPower += teleop.gamepad1.left_stick_y * 0.01 * deltaTime;
                teleop.telemetry.addData("Name", currentMotor.getDeviceName());
                teleop.telemetry.addData("Power", motorPower);

                if (teleop.gamepad1.x) {
                    currentMotor.setPower(motorPower);
                }
                if (teleop.gamepad1.a) {
                    currentMotor.setPower(0);
                }
            }
            else {
                if (teleop.gamepad1.left_bumper && !teleop.gamepad1.left_bumper) {
                    if (teleop.gamepad1.left_bumper && !previousGamepad1.left_bumper && motorIndex > 0) {
                        servoIndex--;
                        updateCurrentServo(servoIndex);
                    }
                    if (teleop.gamepad1.right_bumper && !previousGamepad1.right_bumper && motorIndex < (hw.dcMotors.size() - 1)) {
                        servoIndex++;
                        updateCurrentServo(servoIndex);
                    }
                }

                servoPosition += teleop.gamepad1.right_stick_y * 0.01 * deltaTime;

                teleop.telemetry.addData("Name", currentServo.getDeviceName());
                teleop.telemetry.addData("Position", servoPosition);

                if (teleop.gamepad1.x) {
                    currentServo.setPosition(servoPosition);
                }
            }

            teleop.telemetry.update();
            previousTime = timer.milliseconds();
            try {
                previousGamepad1.copy(teleop.gamepad1);
            }
            catch (RobotCoreException e) {
                RobotLog.e(e.toString());
                teleop.telemetry.addData("ERROR", "Critical error; check log for more details");
            }
        }
    }
}
