/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear Opmode")
public class TelemetryOpmode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    double clawPos1;
    double clawPos2;

    boolean clawEngaged;

    @Override
    public void runOpMode() {

        RobotHardware robot = RobotHardware.GetSingleton(hardwareMap);
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain();

        robot.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // region driving
            double turn = gamepad1.left_trigger - gamepad1.right_trigger;

            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0)) {

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);

                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    drivetrain.FieldOrientedDrive(0, angle, turn);
                }
                else {
                    double speed = Math.abs((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y)) / 2);
                    drivetrain.FieldOrientedDrive(speed, angle, turn);
                }
            }
            else {
                drivetrain.stop();
            }
            //endregion

            if (gamepad2.x && !previousGamepad2.x) {
                clawEngaged = !clawEngaged;
            }

            telemetry.addData("Claw", clawEngaged);

            if (clawEngaged) {
                robot.clawServo1.setPosition(.1);
                robot.clawServo2.setPosition(.8);
            }
            else {
                robot.clawServo1.setPosition(0.4);
                robot.clawServo2.setPosition(0.6);
                clawPos1 = 0.4;
                clawPos2 = 0.6;
            }

            double clawPower = (gamepad2.a ? 1 : 0) - (gamepad2.b ? 1 : 0);

            robot.relicClawServo.setPower(clawPower);

            double beltPower = (gamepad2.left_bumper ? 1 : 0) - (gamepad2.right_bumper ? 1 : 0);

            robot.beltServo.setPower(beltPower);

            double linearSlidePower = gamepad2.left_trigger - gamepad2.right_trigger;

            robot.linearSlideMotor.setPower(linearSlidePower);

            try {
                previousGamepad1.copy(gamepad1);
                previousGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {
                RobotLog.e("Something went wrong while copying gamepads");
            }

            telemetry.update();

            idle();
        }
    }

    public boolean between(double lower, double upper, double value) {
        return (value < upper && value > lower);
    }

    double greater(double a, double b) {
        return (a > b) ? a : b;
    }
}
