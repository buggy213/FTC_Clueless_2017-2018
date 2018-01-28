package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by slau on 1/27/2018.
 */
@TeleOp(name = "Gyrotest", group = "group")
public class GyroTest extends OpMode {
    RobotHardware hw;
    @Override
    public void init() {
        hw = RobotHardware.GetSingleton(hardwareMap);
        hw.ReinitializeIMU();
        hw.imu.startAccelerationIntegration(new Position(), new Velocity(), 16);
    }

    @Override
    public void loop() {
        telemetry.addData("angle", hw.imu.getAngularOrientation().firstAngle);
    }
}
