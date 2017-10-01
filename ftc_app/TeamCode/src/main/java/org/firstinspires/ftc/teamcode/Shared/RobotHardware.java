package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

/**
 * Created by Joshua on 9/9/2017.
 */

public class RobotHardware {

    private static RobotHardware hw;

    private HardwareMap map;

    public static RobotHardware GetSingleton(HardwareMap map) {
        if (hw == null) {
            hw = new RobotHardware(map);
        }

        return hw;
    }

    public static RobotHardware GetSingleton() {
        if (hw == null) {
            throw new RuntimeException();
        }

        return hw;
    }

    public RobotHardware(HardwareMap map) {

    }

}
