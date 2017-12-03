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

    ColorSensor leftColor;
    ColorSensor rightColor;

    ElapsedTime timer;
    DistanceSensor distanceSensor;
    double previousTime;

    double servoPos1 = 0.5;
    double servoPos2;

    @Override
    public void init() {
        timer = new ElapsedTime();
        leftClaw = hardwareMap.servo.get("altClawTurn");
        rightClaw = hardwareMap.servo.get("altClawRight");
        leftColor = hardwareMap.colorSensor.get("left_color");
        rightColor = hardwareMap.colorSensor.get("right_color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    }
    @Override
    public void loop() {
        double deltaTime = (timer.milliseconds() - previousTime) / 20;
        servoPos1 += 0.2 * deltaTime * gamepad1.left_stick_y;
        servoPos2 += 0.2 * deltaTime * gamepad1.right_stick_y;

        leftClaw.setPosition(servoPos1);
        rightClaw.setPosition(servoPos2);

        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Color Sensor", "R-" + leftColor.red() + " G-" + leftColor.green() + " B-" + leftColor.blue());
        telemetry.addData("Right Color Sensor", "R-" + rightColor.red() + " G-" + rightColor.green() + " B-" + rightColor.blue());
        telemetry.addData("Servo 1", servoPos1);
        telemetry.addData("Servo 2", servoPos2);
        telemetry.update();

        previousTime = timer.milliseconds();
    }
}
