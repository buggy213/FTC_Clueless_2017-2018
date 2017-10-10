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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    RobotHardware robot;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    final double speedMultiplier = 0.75;
    final double rotSpeed = 0.5;

    // final double deadZoneX = 0.5;
    // final double deadZoneY = 0.5;

    @Override
    public void runOpMode() {

        robot = RobotHardware.GetSingleton(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* Control Mode 1 - Not working
            float leftPower = 0;
            float rightPower = 0;

            float forwardLeftPower = 0;
            float forwardRightPower = 0;
            float backLeftPower = 0;
            float backRightPower = 0;

            if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {

            }
            else if (gamepad1.left_stick_x == 0) {
                // Moving forwards or backwards
                leftPower = rightPower += gamepad1.left_stick_y;
            }
            else if (gamepad1.left_stick_y == 0) {
                // Moving side to side
                forwardLeftPower = gamepad1.left_stick_x;
                backLeftPower = -gamepad1.left_stick_x;
                forwardRightPower = -gamepad1.left_stick_x;
                backRightPower = gamepad1.left_stick_x;
            }
            else {
                // Moving diagonally
                // TODO this later i'm lazy lel
            }

            float rot = -gamepad1.left_trigger + gamepad1.right_trigger;
            leftPower -= rot;
            rightPower += rot;

            robot.forwardLeft.setPower((forwardLeftPower + leftPower)/2);
            robot.backLeft.setPower((backLeftPower + leftPower)/2);

            robot.forwardRight.setPower((forwardRightPower + rightPower)/2);
            robot.backRight.setPower((backRightPower + rightPower)/2);
            */
            // boolean left = between(-deadZoneX, deadZoneX, gamepad1.left_stick_x);
            // boolean right = between(-deadZoneX, deadZoneX, gamepad1.right_stick_x);

            double vRot = rotSpeed * (-gamepad1.left_trigger + gamepad1.right_trigger);

            double speed = greater(Math.abs(gamepad1.right_stick_y), Math.abs(gamepad1.left_stick_x));
            speed = Math.abs(speed * speedMultiplier);

            double desiredAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.left_stick_x) ;
            if (desiredAngle < 0) {
                desiredAngle = desiredAngle + 2 * 3.14;
            }

            telemetry.addData("Desired degrees (CCW)", desiredAngle * 180 / 3.14);

            double intermediateSin = Math.sin(desiredAngle + 0.79);
            double intermediateCos = Math.cos(desiredAngle + 0.79);

            double leftForward = speed * (intermediateSin) + vRot;
            double leftBackward = speed * (intermediateCos) + vRot;
            double rightForward = speed * (intermediateCos) - vRot;
            double rightBackward = speed * (intermediateSin) - vRot;

            /*if (left && right) {
                // Both forward / backward
                leftForward = gamepad1.left_stick_y;
                leftBackward = gamepad1.left_stick_y;

                rightForward = gamepad1.right_stick_y;
                rightBackward = gamepad1.right_stick_y;
            }
            else if (left || right) {
                // One side, one forward
                // Moving diagonally
                if (left) {
                    // left is forward and backward
                    // right is left and right

                }
                else {
                    // right is forward and backward
                    // left is left and right
                }
            }
            else {
                double power = (gamepad1.left_stick_x + gamepad1.right_stick_x) / 2;
                leftForward = power;
                rightBackward = power;
                rightForward = -power;
                leftBackward = -power;
            }*/

            robot.forwardLeft.setPower(leftForward);
            robot.forwardRight.setPower(rightForward);
            robot.backLeft.setPower(leftBackward);
            robot.backRight.setPower(rightBackward);

            telemetry.addData("Left stick x and y", gamepad1.left_stick_x + ", " + gamepad1.left_stick_y);
            telemetry.addData("Right stick x and y", gamepad1.right_stick_x + ", " + gamepad1.right_stick_y);

            telemetry.addData("Left forward power", leftForward);
            telemetry.addData("Left backward power", leftBackward);
            telemetry.addData("Right forward power", rightForward);
            telemetry.addData("Right backward power", rightBackward);


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
