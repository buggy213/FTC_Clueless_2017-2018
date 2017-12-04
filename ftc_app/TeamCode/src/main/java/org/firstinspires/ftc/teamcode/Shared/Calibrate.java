package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by hsunx on 11/24/2017.
 */
// Calibrates BNO1055 sensor built into the REV Expansion Hub, takes on average 20-40 seconds
    @Autonomous(name = "Calibrate", group = "prematch")
public class Calibrate extends OpMode {

    @Override
    public void init() {
        RobotHardware robot = RobotHardware.GetSingleton(hardwareMap);
        if (robot.imu == null) {
            robot.ReinitializeIMU();
        }
        // No point in reinitializing if it's already there...
    }

    @Override
    public void loop() {

    }
}
