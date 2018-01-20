package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by hsunx on 11/26/2017.
 */
@TeleOp(name = "Test Parking", group = "test")
public class park_test extends LinearOpMode {
    double encoderCount = 855;

    ElapsedTime timer;
    double previousTime;

    final double turnSpeed = 0.6;
    final double fastSpeed = 0.85;
    final double normalSpeed = 0.4;
    final double slowSpeed = 0.4;
    final double slowestSpeed = 0.2;
    double horizotalSpeedMultiplier = 1;

    //RobotHardware hw = RobotHardware.GetSingleton(hardwareMap);
    //FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain();
    RobotHardware hw;

    FourWheelMecanumDrivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        hw = RobotHardware.GetSingleton(hardwareMap);
        RobotHardware.SetCurrentRunningOpMode(this);

        drivetrain = new FourWheelMecanumDrivetrain();
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {
            double deltaTime = (timer.milliseconds() - previousTime) / 40;

            double turn = -1 * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0) ) {

                double speed = 1;
                if (gamepad1.right_stick_y == 0) {
                    speed = 1.25 * horizotalSpeedMultiplier ;
                }

                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    speed = 0;
                }

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                drivetrain.MoveAngle(speed, angle , turn);
            } else {
                drivetrain.stop();
            }

            if (gamepad1.a) {
                encoderCount += 100 * deltaTime;
            }

            if (gamepad1.b) {
                encoderCount -= 100 * deltaTime ;
            }

            if (gamepad1.x) {
                AutoMove(slowSpeed, 0, (int)encoderCount);
            }


            if (gamepad1.dpad_left || gamepad1.left_bumper) {
                drivetrain.setSpeedMultiplier(fastSpeed);
                horizotalSpeedMultiplier = 1;
            }

            if (gamepad1.dpad_up) {
                drivetrain.setSpeedMultiplier(normalSpeed);
                horizotalSpeedMultiplier = 1.4;
            }

            if (gamepad1.right_bumper) {
                drivetrain.setSpeedMultiplier(slowSpeed);
                horizotalSpeedMultiplier = 1;
            }

            if (gamepad1.dpad_right) {
                drivetrain.setSpeedMultiplier(slowestSpeed);
                horizotalSpeedMultiplier = 1.55;
            }

            double linearSlideDrivePower = gamepad2.left_stick_y + gamepad2.right_stick_y;
            hw.linearSlideDriveMotor.setPower(linearSlideDrivePower);

            telemetry.addData("Encoder Count", encoderCount);
            telemetry.addData("deltaTime", deltaTime);
            telemetry.update();
            previousTime = timer.milliseconds();
        }
    }

    public void AutoMove(double speed, double angle, int counts) {
        int initialForwardLeft = hw.forwardLeft.getCurrentPosition();
        int initialForwardRight = hw.forwardRight.getCurrentPosition();
        int initialBackwardLeft = hw.backLeft.getCurrentPosition();
        int initialBackwardRight = hw.backRight.getCurrentPosition();

        drivetrain.setSpeedMultiplier(slowSpeed);
        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive()) {
            int differenceForwardLeft = Math.abs(hw.forwardLeft.getCurrentPosition() - initialForwardLeft);
            int differenceForwardRight = Math.abs(hw.forwardRight.getCurrentPosition() - initialForwardRight);
            int differenceBackwardLeft = Math.abs(hw.backLeft.getCurrentPosition() - initialBackwardLeft);
            int differenceBackwardRight= Math.abs(hw.backRight.getCurrentPosition() - initialBackwardRight);
            if ((differenceBackwardLeft + differenceForwardLeft + differenceBackwardRight + differenceBackwardRight) / 4 > counts) {
                drivetrain.stop();
                break;
            }
        }
    }
}
