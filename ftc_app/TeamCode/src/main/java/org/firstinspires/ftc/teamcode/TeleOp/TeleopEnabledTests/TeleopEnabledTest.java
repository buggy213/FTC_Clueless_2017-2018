package org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Joshua on 2/8/2018.
 */

public abstract class TeleopEnabledTest {
    LinearOpMode teleop;

    public TeleopEnabledTest() {

    }
    public TeleopEnabledTest(LinearOpMode opMode) {
        this.teleop = opMode;
    }

    public abstract void run();
}
