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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests.TeleopEnabledTest;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests.TeleopEnabledTestRegistrar;

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

@TeleOp(name="Testing Program", group="Linear Opmode")
public class TestingTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    // Constants for teleop
    final double turnSpeed = 0.6;
    final double fastSpeed = 1;
    final double slowSpeed = 0.8;
    boolean reverse = false;
    boolean testing = true;
    int altClawPosition = 1;
    int upperClawPosition = 0;
    int altClawTurned = 1;
    int index = 0;
    boolean turningTowards = false;

    TeleopEnabledTest test;

    FourWheelMecanumDrivetrain drivetrain;
    RobotHardware robot;

    public void setTest(int index) {
        try {
            test = TeleopEnabledTestRegistrar.teleopEnabledTestClasses.get(index).newInstance();
            test.setOpMode(this);
            test.init();
        }
        catch (Exception e) {
            RobotLog.e(e.toString());
            telemetry.addData("ERROR", "Critical error in instantiating new test, check log for details");
            telemetry.update();
            sleep(1500);
            requestOpModeStop();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TeleopEnabledTestRegistrar.LoadTests();
        robot = RobotHardware.GetSingleton(hardwareMap);
        drivetrain = new FourWheelMecanumDrivetrain();

        robot.jewelArm1.setPosition(0.18);  //0.15
        robot.jewelArm2.setPosition(0.92);   //0.85
        robot.leftFlick.setPosition(0.61);
        robot.rightFlick.setPosition(0.35);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setRunningOpMode(this);

        robot.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.linearSlidePivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setSpeedMultiplier(slowSpeed);
        drivetrain.resetEncoders();

        robot.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        setTest(0);

        robot.altClawTurn.setPosition(0.5);  // center

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Driving Gamepads logic
            // region driving
            double turn = ((reverse) ? 1 : -1) * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

            if (gamepad1.left_stick_button) {
                drivetrain.EncoderTurn(0.15, 800, false);
            }
            if (gamepad1.right_stick_button) {
                drivetrain.Rotate(true, 0.15);
            }

            if (gamepad1.a) {
                if (index > 0) {
                    index--;
                    setTest(index);
                }
            }
            if (gamepad1.b) {
                if (index < (TeleopEnabledTestRegistrar.teleopEnabledTestClasses.size() - 1)) {
                    index++;
                    setTest(index);
                }
            }

            if (gamepad1.dpad_left) {
                testing = false;
            }
            if (gamepad1.dpad_right) {
                testing = true;
            }

            telemetry.addData("Name", test.getClass().getSimpleName());
            telemetry.addData("Description", test.description);

            if (gamepad1.x) {
                try {
                    testing = true;
                    test.run();
                }
                catch (Exception e){
                    // WCGW? Apparently things
                    telemetry.addData("Issue", e.toString());
                    RobotLog.e(e.toString());
                    telemetry.update();
                }
            }
            if (!testing) {
                if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0) && !turningTowards) {

                    double speed = 1;
                /* if (gamepad1.right_stick_y == 0) {
                    speed = 1.25 * horizotalSpeedMultiplier ;
                }
                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    speed = 0;
                } */

                    if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                        speed = 0;
                    } else if (gamepad1.right_stick_y == 0) {
                        speed = Math.abs(gamepad1.left_stick_x);
                    } else if (gamepad1.left_stick_x == 0) {
                        speed = Math.abs(gamepad1.right_stick_y);
                    } else {
                        speed = (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y)) / 2;
                    }

                    double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                    drivetrain.MoveAngle(speed, angle + ((reverse) ? Math.PI : 0), turn);
                } else {
                    drivetrain.stop();
                }

                if (gamepad2.dpad_right) {
                    altClawPosition = 0;  // Complete closed
                    upperClawPosition = 0; // open upperClaw
                }
                if (gamepad2.dpad_left) {
                    altClawPosition = 1;  // open altClaw
                    upperClawPosition = 0; // open upperClaw
                }
                if (gamepad2.dpad_up) {  // Grabbing two glyphs
                    altClawPosition = 2;
                    upperClawPosition = 0; // open upperClaw
                }
                if (gamepad2.dpad_down) {  // Grabbing one glyphs
                    altClawPosition = 3;
                    if (altClawTurned == 1) { // unless altClaw is in the center position, disallowed upperClaw to be engaged
                        upperClawPosition = 1;
                    }
                }

                if ((gamepad2.x) && (altClawTurned == 1)) {  // unless altClaw is in the center position, disallowed upperClaw to be engaged
                    upperClawPosition = 1;
                }
                if (gamepad2.y) {
                    upperClawPosition = 0;
                }

            /*if (gamepad1.a) {
                reverse = true;
            }

            if (gamepad1.b) {
                reverse = false;
            }*/

            /* turningTowards = gamepad1.x;
            if (gamepad1.x) {

                drivetrain.GyroTurnTeleop(0.2, 90);
            } */

                if (gamepad2.a) {
                    altClawPosition = 4; // slight open
                    upperClawPosition = 2;  // slight open
                }
                //if (gamepad2.b) {
                //    upperClawPosition = 2 ;  // slight open
                //}

                if ((gamepad2.left_bumper && gamepad2.right_bumper) || gamepad2.b) {
                    robot.altClawTurn.setPosition(0.5);  // center
                    altClawTurned = 1;  // center
                } else if (gamepad2.left_bumper) {
                    // if turning altClaw, make sure upperClaw is open before turning
                    robot.upperLeft.setPosition(0.19);
                    robot.upperRight.setPosition(0.77);
                    upperClawPosition = 0;
                    //wait(200);

                    robot.altClawTurn.setPosition(0.14); // left turn
                    altClawTurned = 0;  // left turn
                } else if (gamepad2.right_bumper) {
                    // if turning altClaw, make sure upperClaw is open
                    robot.upperLeft.setPosition(0.19);
                    robot.upperRight.setPosition(0.77);
                    upperClawPosition = 0;
                    //wait(200);

                    robot.altClawTurn.setPosition(0.87);  // right turn
                    altClawTurned = 2;  // right turn
                }

                switch (altClawPosition) {
                    case 0: // Complete closed
                        robot.altClawLeft.setPosition(0.88);
                        robot.altClawRight.setPosition(0.12);
                        break;
                    case 1:  // open altClaw
                        if (altClawTurned == 1) {  // center
                            // Ready to grab
                            robot.altClawLeft.setPosition(0.29);
                            robot.altClawRight.setPosition(0.65);
                        }
                        // releasing left or right altClaw depending on the turn position
                        else if (altClawTurned == 0) {
                            // Right releasing
                            robot.altClawLeft.setPosition(0.29);  // Keep this same as two-glyph setting or center
                            robot.altClawRight.setPosition(0.84); //
                        } else if (altClawTurned == 2) {
                            // Left releasing
                            robot.altClawLeft.setPosition(0.13);  //
                            robot.altClawRight.setPosition(0.65);  // Keep this same as two-glyph setting or center
                        }
                        break;
                    case 2: // Grabbing two glyphs
                        robot.altClawLeft.setPosition(0.468);
                        robot.altClawRight.setPosition(0.47);
                        break;
                    case 3: // Grabbing one glyphs
                        robot.altClawLeft.setPosition(0.67);  //0.610
                        robot.altClawRight.setPosition(0.30);  //0.276
                        break;
                    case 4: // slight open
                        // release altClawTurned and upperClaw (in vertical glyph positions
                        robot.altClawLeft.setPosition(0.556); //0.550
                        robot.altClawRight.setPosition(0.36); //0.336
                        break;
                }

                switch (upperClawPosition) {
                    case 0: // Resting
                        robot.upperLeft.setPosition(0.19);
                        robot.upperRight.setPosition(0.77);
                        break;
                    case 1: // Grabbing
                        robot.upperLeft.setPosition(0.61);  //0.6
                        robot.upperRight.setPosition(0.33);  //0.36
                        break;
                    case 2:  // slight open
                        robot.upperLeft.setPosition(0.55);
                        robot.upperRight.setPosition(0.39);
                        break;
                }

                if (gamepad1.left_bumper) {
                    drivetrain.setSpeedMultiplier(fastSpeed);
                }
                if (gamepad1.right_bumper) {
                    drivetrain.setSpeedMultiplier(slowSpeed);
                }

                if (gamepad2.guide || gamepad2.back) {
                    robot.jewelArm1.setPosition(0.18);
                    robot.jewelArm2.setPosition(0.85);
                }

                // double linearSlidePivotPower = (gamepad1.left_stick_button ? 1 : 0) + (gamepad1.right_stick_button ? -1 : 0);

                // robot.linearSlidePivotMotor.setPower(linearSlidePivotPower);

                double linearSlideDrivePower = gamepad2.left_stick_y + gamepad2.right_stick_y;

                robot.linearSlideDriveMotor.setPower(linearSlideDrivePower);

            /*try {
                previousGamepad1.copy(gamepad1);
            }

            catch (RobotCoreException e) {
                RobotLog.e("Something went wrong while copying gamepads");
            } */

                // telemetry.addData("Heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                // telemetry.addData("altClawTurned", altClawTurned);
                // telemetry.addData("upperClawPosition", upperClawPosition);
                // telemetry.addData("altClawPosition", altClawPosition);
                // telemetry.addData("normalBumper", normalBumper);
                // telemetry.addData("FL", robot.forwardLeft.getCurrentPosition());
                // telemetry.addData("FR", robot.forwardRight.getCurrentPosition());
                // telemetry.addData("BL", robot.backLeft.getCurrentPosition());
                // telemetry.addData("BR", robot.backRight.getCurrentPosition());
            }
            telemetry.update();
        }
    }

    private void resetJewelArms() {

    }

    public void AutoMove(double speed, double angle, int counts) {
        int initialForwardLeft = robot.forwardLeft.getCurrentPosition();
        int initialForwardRight = robot.forwardRight.getCurrentPosition();
        int initialBackwardLeft = robot.backLeft.getCurrentPosition();
        int initialBackwardRight = robot.backRight.getCurrentPosition();

        drivetrain.setSpeedMultiplier(slowSpeed);
        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive()) {
            int differenceForwardLeft = Math.abs(robot.forwardLeft.getCurrentPosition() - initialForwardLeft);
            int differenceForwardRight = Math.abs(robot.forwardRight.getCurrentPosition() - initialForwardRight);
            int differenceBackwardLeft = Math.abs(robot.backLeft.getCurrentPosition() - initialBackwardLeft);
            int differenceBackwardRight= Math.abs(robot.backRight.getCurrentPosition() - initialBackwardRight);
            if ((differenceBackwardLeft + differenceForwardLeft + differenceBackwardRight + differenceBackwardRight) / 4 > counts) {
                drivetrain.stop();
                break;
            }
            if (gamepad1.y) {
                drivetrain.stop();
                break;
            }
        }
    }

    public boolean between(double lower, double upper, double value) {
        return (value < upper && value > lower);
    }

    double greater(double a, double b) {
        return (a > b) ? a : b;
    }
}