package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by slau on 12/27/2017.
 */
@Autonomous(name="DistanceSensor Test", group="test")
public class VL53L0X_Test extends OpMode {
    LVEZ4 device;

    @Override
    public void init() {
        device = new LVEZ4(hardwareMap.analogInput.get("ultrasonic"));
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", device.distance());
        telemetry.addData("Voltage", device.getInput().getVoltage());
    }
}
