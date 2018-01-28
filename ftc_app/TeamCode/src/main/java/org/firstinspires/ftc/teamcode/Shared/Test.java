package org.firstinspires.ftc.teamcode.Shared;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Test.CryptoboxDetector;
import org.firstinspires.ftc.teamcode.Test.TeamColor;

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
    //double servoPos3 = 0.5;

    @Override
    public void init() {
        timer = new ElapsedTime();
        CryptoboxDetector detector = new CryptoboxDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, TeamColor.RED);
        detector.enable();
        leftClaw = hardwareMap.servo.get("phoneServo1");
        rightClaw = hardwareMap.servo.get("phoneServo2");
        turnClaw = hardwareMap.servo.get("altClawTurn");
        //leftColor = hardwareMap.colorSensor.get("left_color");
        //rightColor = hardwareMap.colorSensor.get("right_color");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    }

    @Override

    public void loop() {
        double deltaTime = (timer.milliseconds() - previousTime) / 40;
        servoPos1 += 0.05 * deltaTime * gamepad1.left_stick_y;
        servoPos2 += 0.05 * deltaTime * gamepad1.right_stick_y;
        //servoPos3 += 0.05 * deltaTime * gamepad1.left_trigger;

        leftClaw.setPosition(servoPos1);
        rightClaw.setPosition(servoPos2);
        //turnClaw.setPosition(servoPos3);
        telemetry.addData("Servo 1", servoPos1);
        telemetry.addData("Servo 2", servoPos2);
        //telemetry.addData("Servo 3", servoPos3);
        //telemetry.addData("Right RGBA", rightColor.red() + ", " + rightColor.green() + ", " + rightColor.blue());
        //telemetry.addData("Left RGBA", leftColor.red() + ", " + leftColor.green() + ", " + leftColor.blue());
        telemetry.update();

        previousTime = timer.milliseconds();
    }
}
