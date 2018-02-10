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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Shared.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;
import org.firstinspires.ftc.teamcode.Test.CryptoboxDetector;
import org.firstinspires.ftc.teamcode.Test.TeamColor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Experimental Autonomous", group = "Linear Opmode")
public class ExperimentalAutonomousOpMode extends LinearOpMode {

    RobotHardware hw;

    FourWheelMecanumDrivetrain drivetrain;

    CryptoboxDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = RobotHardware.GetSingleton(hardwareMap);
        RobotHardware.SetCurrentRunningOpMode(this);
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain = new FourWheelMecanumDrivetrain();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        JewelDetector jewels = new JewelDetector();
        jewels.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewels.areaWeight = 0.02;
        jewels.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewels.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewels.debugContours = true;
        jewels.maxDiffrence = 15;
        jewels.ratioWeight = 15;
        jewels.minArea = 700;

        jewels.enable();

        int times = 0; // Make sure the jewel detector is confident
        JewelDetector.JewelOrder order = JewelDetector.JewelOrder.UNKNOWN;
        while(true) {
            if (jewels.getCurrentOrder() != JewelDetector.JewelOrder.UNKNOWN) {
                if (order != jewels.getCurrentOrder()) {
                    order = jewels.getCurrentOrder();
                    times = 0;
                }
                else {
                    times++;
                }
            }
            else {
                times = 0;
            }
            if (times > 10) {
                break;
            }
            telemetry.addData("Order: ", order.toString());
            telemetry.addData("Reliability Check", times + "/10");
            telemetry.update();
            this.sleep(250);
        }

        jewels.disable();

        detector = new CryptoboxDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, TeamColor.BLUE);
        detector.enable();
        detector.passThrough = false;
        waitForStart();

        visionCrypto(0.1, 0.0003, Direction.RIGHT, Direction.FORWARD);

        detector.disable();
    }

    private void visionCrypto(double speed, double p, Direction direction, Direction desired) {
        boolean blue = direction == Direction.RIGHT;
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();

        int id1 = 0;
        int id2 = 0;
        boolean specialMode = false;
        if (blue) {
            switch(desired) {
                case LEFT:
                    specialMode = true;
                    break;
                case RIGHT:
                    id1 = 1;
                    id2 = 2;
                    break;
                case FORWARD:
                    id1 = 0;
                    id2 = 1;
                    break;
            }
        }
        else {
            switch(desired) {
                case LEFT:
                    id1 = 1;
                    id2 = 2;
                    break;
                case RIGHT:
                    specialMode = true;
                    break;
                case FORWARD:
                    id1 = 0;
                    id2 = 1;
                    break;
            }
        }
        if (specialMode) {
            throw new RuntimeException("Not implemented yet");
        }
        else {
            while (opModeIsActive()) {

                if (detector.getCenterPoint(id1, id2) != -1) {
                    telemetry.addData("Detected -- Screen Fraction", detector.getCenterPoint(id1, id2) / detector.frameSize.width);
                    RobotLog.i(String.valueOf(detector.getCenterPoint(id1, id2) / detector.frameSize.width));
                    telemetry.update();
                    // Two desired columns are detected
                    if (direction == Direction.LEFT) {
                        if (detector.getCenterPoint(id1, id2) / detector.frameSize.width > 0.40) {
                            break;
                        }
                    }
                    else {
                        if (detector.getCenterPoint(id1, id2) / detector.frameSize.width < 0.60) {
                            break;
                        }
                    }
                }
                int backLeft = hw.backLeft.getCurrentPosition();
                int backRight = hw.backRight.getCurrentPosition();
                int forwardLeft = hw.forwardLeft.getCurrentPosition();
                int forwardRight = hw.forwardRight.getCurrentPosition();

                int backLeftDiff = Math.abs(backLeft - backLeftStart);
                int backRightDiff = Math.abs(backRight - backRightStart);
                int forwardLeftDiff = Math.abs(forwardLeft - forwardLeftStart);
                int forwardRightDiff = Math.abs(forwardRight - forwardRightStart);

                double avg = (backLeftDiff + backRightDiff + forwardLeftDiff + forwardRightDiff) / 4;
                double backLeftComp = (avg - backLeftDiff) * p;
                double backRightComp = (avg - backRightDiff) * p;
                double forwardLeftComp = (avg - forwardLeftDiff) * p;
                double forwardRightComp = (avg - forwardRightDiff) * p;

                switch (direction) {
                    case LEFT:
                        hw.forwardRight.setPower(speed + forwardRightComp);
                        hw.forwardLeft.setPower(-speed - forwardLeftComp);
                        hw.backRight.setPower(-speed - backRightComp);
                        hw.backLeft.setPower(speed + backLeftComp);
                        break;
                    case RIGHT:
                        hw.forwardRight.setPower(-speed - forwardRightComp);
                        hw.forwardLeft.setPower(speed + forwardLeftComp);
                        hw.backRight.setPower(speed + backRightComp);
                        hw.backLeft.setPower(-speed - backLeftComp);
                        break;

                }
            }

            drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drivetrain.stop();
        }
    }
}