package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by slau on 12/27/2017.
 */
@Autonomous(name="DistanceSensor Test", group="test")
public class VL53L0X_Test extends OpMode {
    VL53L0X device;

    @Override
    public void init() {
        device = hardwareMap.get(VL53L0X.class, "distanceSensor");
    }

    @Override
    public void loop() {
        try {
            device.startRanging(VL53L0X.VL53L0X_BEST_ACCURACY_MODE);
        }
        catch(Exception e) {
            RobotLog.e(e.toString());
        }
        try {
            telemetry.addData("Distance", device.getDistance());
        }
        catch (Exception e) {
            RobotLog.e(e.toString());
        }

        telemetry.update();
    }
}
