package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by hsunx on 11/24/2017.
 */

public class Calibrate extends OpMode {


    @Override
    public void init() {
        RobotHardware robot = RobotHardware.GetSingleton(hardwareMap);
        robot.ReinitializeIMU();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 150);
    }

    @Override
    public void loop() {

    }
}
