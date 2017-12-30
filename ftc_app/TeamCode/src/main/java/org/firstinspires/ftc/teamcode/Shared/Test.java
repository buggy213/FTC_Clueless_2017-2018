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
    //DistanceSensor distanceSensor;
    double previousTime;

    double servoPos1 = 0.5;
    double servoPos2 = 0.5;

    @Override
    public void init() {
        timer = new ElapsedTime();
        //leftClaw = hardwareMap.servo.get("altClawLeft");
        //rightClaw = hardwareMap.servo.get("altClawRight");
        leftClaw = hardwareMap.servo.get("jewelArm1");
        rightClaw = hardwareMap.servo.get("jewelArm2");
        leftColor = hardwareMap.colorSensor.get("left_color");
        rightColor = hardwareMap.colorSensor.get("right_color");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    }
    @Override
    public void loop() {
        double deltaTime = (timer.milliseconds() - previousTime) / 40;
        servoPos1 += 0.05 * deltaTime * gamepad1.left_stick_y;
        servoPos2 += 0.05 * deltaTime * gamepad1.right_stick_y;

        leftClaw.setPosition(servoPos1);
        rightClaw.setPosition(servoPos2);
        telemetry.addData("Servo 1", servoPos1);
        telemetry.addData("Servo 2", servoPos2);
        telemetry.update();

        previousTime = timer.milliseconds();
    }
}
