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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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

    // Constants for teleop
    final double turnSpeed = 0.6;
    final double slowSpeed = 0.4;

    int index;

    boolean upperClawEngaged = false;
    boolean lowerClawEngaged = false;

    boolean altClawTurned = false;

    boolean slowMode = true;

    @Override
    public void runOpMode() {

        RobotHardware robot = RobotHardware.GetSingleton(hardwareMap);
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain();


        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setSpeedMultiplier(slowSpeed);

        robot.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Driving/Gamepads logic
            // region driving
            double turn = (gamepad1.right_trigger - gamepad1.left_trigger) * turnSpeed;

            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0)) {

                double speed = 1;
                if (gamepad1.right_stick_y == 0) {
                    speed = 1.25;
                }

                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    speed = 0;
                }

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                drivetrain.MoveAngle(speed, angle, turn);
            }
            else {
                drivetrain.stop();
            }
            //endregion

            if (!previousGamepad2.a && gamepad2.a) {
                upperClawEngaged = !upperClawEngaged;
            }
            if (!previousGamepad2.b && gamepad2.b) {
                lowerClawEngaged = !lowerClawEngaged;
            }

            if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
                index++;
                index = index % 3;
            }
            if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
                index--;
                if (index < 0) {
                    index = 2;
                }
            }

            switch (index) {
                case 0:
                    // Resting
                    robot.altClawLeft.setPosition(0.864);
                    robot.altClawRight.setPosition(0.079);
                    break;
                case 1:
                    // Ready to grab
                    robot.altClawLeft.setPosition(0.127);
                    robot.altClawRight.setPosition(0.839);
                    break;
                case 2:
                    // Grabbing
                    robot.altClawLeft.setPosition(0.374);
                    robot.altClawRight.setPosition(0.555);
                    break;
            }

            if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
                altClawTurned = !altClawTurned;
            }

            if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
                index = 0;
            }

            if (gamepad2.x) {
                upperClawEngaged = false;
                lowerClawEngaged = false;
            }
            if (gamepad2.y) {
                upperClawEngaged = true;
                lowerClawEngaged = true;
            }
            if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = !slowMode;
                if (slowMode) {
                    drivetrain.setSpeedMultiplier(slowSpeed);
                }
                else {
                    drivetrain.setSpeedMultiplier(1);
                }
            }

            if (lowerClawEngaged) {
                robot.lowerLeft.setPosition(0.419);
                robot.lowerRight.setPosition(0.491);
            }
            else {
                robot.lowerLeft.setPosition(1);
                robot.lowerRight.setPosition(0);
            }

            if (upperClawEngaged) {
                robot.upperLeft.setPosition(0.419);
                robot.upperRight.setPosition(0.491);
            }
            else {
                robot.upperLeft.setPosition(1);
                robot.upperRight.setPosition(0);
            }

            if (altClawTurned) {
                robot.altClawTurn.setPosition(0.946);
            }
            else {
                robot.altClawTurn.setPosition(0.430);
            }

            double linearSlidePivotPower = (gamepad1.left_stick_button ? 1 : 0) + (gamepad1.right_stick_button ? -1 : 0);

            robot.linearSlidePivotMotor.setPower(linearSlidePivotPower);

            double linearSlideDrivePower = gamepad2.left_stick_y;

            robot.linearSlideDriveMotor.setPower(linearSlideDrivePower);

            try {
                previousGamepad1.copy(gamepad1);
                previousGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {
                RobotLog.e("Something went wrong while copying gamepads");
            }
            // telemetry.addData("Heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }
    }

    public boolean between(double lower, double upper, double value) {
        return (value < upper && value > lower);
    }

    double greater(double a, double b) {
        return (a > b) ? a : b;
    }
}
