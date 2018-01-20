package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by hsunx on 11/26/2017.
 */
@TeleOp(name = "Test OpMode", group = "test")
public class Test extends OpMode {
    Servo leftClaw;
    Servo rightClaw;
    Servo turnClaw;

    ColorSensor leftColor;
    ColorSensor rightColor;

    ElapsedTime timer;
    //DistanceSensor distanceSensor;
    double previousTime;

    double servoPos1 = 0.5;
    double servoPos2 = 0.5;
    double servoPos3 = 0.5;

    @Override
    public void init() {
        timer = new ElapsedTime();
        leftClaw = hardwareMap.servo.get("altClawLeft");
        rightClaw = hardwareMap.servo.get("altClawRight");
        turnClaw = hardwareMap.servo.get("altClawTurn");
        leftColor = hardwareMap.colorSensor.get("left_color");
        rightColor = hardwareMap.colorSensor.get("right_color");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    }

    @Override

    public void loop() {
        double deltaTime = (timer.milliseconds() - previousTime) / 40;
        servoPos1 += 0.05 * deltaTime * gamepad1.left_stick_y;
        servoPos2 += 0.05 * deltaTime * gamepad1.right_stick_y;
        servoPos3 += 0.05 * deltaTime * gamepad1.left_trigger;

        leftClaw.setPosition(servoPos1);
        rightClaw.setPosition(servoPos2);
        turnClaw.setPosition(servoPos3);
        telemetry.addData("Servo 1", servoPos1);
        telemetry.addData("Servo 2", servoPos2);
        telemetry.addData("Servo 3", servoPos3);
        telemetry.addData("Right RGBA", rightColor.red() + ", " + rightColor.green() + ", " + rightColor.blue());
        telemetry.addData("Left RGBA", leftColor.red() + ", " + leftColor.green() + ", " + leftColor.blue());
        telemetry.update();

        previousTime = timer.milliseconds();
    }
}
