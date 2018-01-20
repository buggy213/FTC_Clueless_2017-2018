package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by slau on 1/13/2018.
 */
@TeleOp(name = "Set Parameters", group = "autoassist")
public class SetParams extends OpMode {
    String setting =
            "none";

    @Override
    public void init() {
        MatchParameters.initCurrentParams();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            setting = "RED_CLOSE";
        }
        if (gamepad1.b) {
            setting = "RED_FAR";
        }
        if (gamepad1.x) {
            setting = "BLUE_CLOSE";
        }
        if (gamepad1.y) {
            setting = "BLUE_FAR";
        }

        telemetry.addData("Current: ", setting);
        telemetry.update();
    }

    @Override
    public void stop() {
        MatchParameters.current.put("start", setting);
        super.stop();
    }
}
