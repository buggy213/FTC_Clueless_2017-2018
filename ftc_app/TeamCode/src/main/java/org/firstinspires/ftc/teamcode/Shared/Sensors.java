package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by slau on 12/3/2017.
 */
@TeleOp
public class Sensors extends OpMode {
    ColorSensor leftColor;
    ColorSensor rightColor;

    DistanceSensor distanceSensor;

    @Override
    public void init() {
        leftColor = hardwareMap.colorSensor.get("left_color");
        rightColor = hardwareMap.colorSensor.get("right_color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Color Sensor", "R-" + leftColor.red() + " G-" + leftColor.green() + " B-" + leftColor.blue());
        telemetry.addData("Right Color Sensor", "R-" + rightColor.red() + " G-" + rightColor.green() + " B-" + rightColor.blue());
        telemetry.update();
    }
}
