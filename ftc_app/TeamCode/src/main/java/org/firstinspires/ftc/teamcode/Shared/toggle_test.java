package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by slau on 12/8/2017.
 */
@TeleOp(name = "toggle")
public class toggle_test extends LinearOpMode {
    Gamepad previousGamepad1 = new Gamepad();

    boolean toggle;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        RobotHardware robot = RobotHardware.GetSingleton(hardwareMap);
        while (opModeIsActive()) {
            telemetry.addData("FL", robot.forwardLeft.getCurrentPosition());
            telemetry.addData("FR", robot.forwardRight.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
