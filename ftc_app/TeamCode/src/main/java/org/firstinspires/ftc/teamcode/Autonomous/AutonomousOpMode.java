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

import android.graphics.Bitmap;
import android.util.Pair;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Shared.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;
import org.firstinspires.ftc.teamcode.Test.CryptoboxDetector;
import org.firstinspires.ftc.teamcode.Test.TeamColor;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import static java.util.Collections.max;


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

@Autonomous(name = "Autonomous", group = "Linear Opmode")
public class AutonomousOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    final int maxBufferResults = 25;

    // Used to access robot hardware
    RobotHardware hw;

    FourWheelMecanumDrivetrain drivetrain;

    // Stores instance of Vuforia Localizer
    ClosableVuforiaLocalizer vuforia;


    JewelDetector jewels = new JewelDetector();

    // Instance of relic trackable
    VuforiaTrackable relicTemplate;

    RelicRecoveryVuMark lastKnownVumark = RelicRecoveryVuMark.UNKNOWN;
    JewelDetector.JewelOrder order;
    CryptoboxDetector detector;

    LinkedList<Pair<RelicRecoveryVuMark, JewelDetector.JewelOrder>> resultsBuffer = new LinkedList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        hw = RobotHardware.GetSingleton(hardwareMap);
        RobotHardware.SetCurrentRunningOpMode(this);
        String mode = "";
        MatchParameters parameters = MatchParameters.current;
        if (parameters == null) {
            requestOpModeStop();
        }
        try {
            mode = parameters.get("start");
        } catch (Exception e) {
            requestOpModeStop();
        }
        boolean close = mode.contains("CLOSE");

        boolean red = mode.contains("RED");
        drivetrain = new FourWheelMecanumDrivetrain();
        hw.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (red) {
            hw.phoneServo2.setPosition(0.67); // 0.64
        } else {
            hw.phoneServo1.setPosition(0.16);  // 0.27
        }
        resetJewelArms();

        /*if (hw.imu == null) {
            hw.ReinitializeIMU();
        }
        hw.imu.startAccelerationIntegration(new Position(), new Velocity(), 16);*/
        resetJewelArms();

        //region Vuforia/Vision
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Jewel Detector Settings
        jewels.areaWeight = 0.02;
        jewels.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewels.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewels.debugContours = true;
        jewels.maxDiffrence = 15;
        jewels.ratioWeight = 15;
        jewels.minArea = 700;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = "ASK0CtX/////AAAAGbyNvvxXMUrohVJwXgMBW5Iqmz//UVeASb2KC//wHTXPuoN/gOM0vbw91nX++j+iS98pzeEfO+p9jpijt7j6VgQZlFTO9K2HjwAvTmG5M6CYglrh0B3kfA/nZx/NSyyxWIRe7Q03DeNDH50ZnSJ3I4FkyD7AbcTbJHg3LjL72N6/Lfm5biUbhOPoeQb1a8qUaqp1Il340pGFEvIEH8s7nhqHAga3TSdvM7yWxqRtZ3Bv2yEmIFMIWuBdEV6ahooWDsnnRmSn33bQVRD+KTNNocJWkwQ1pDG/8XBzswiICKBvcCvzklZhV/TeoJDk6tagk8sMcYxBdzMtyBlYVA9y3iJRyuzu+UMfswmSlA6CnmM/";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(vuforiaParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        //endregion

        waitForStart();

        Runnable motorLift = new Runnable() {
            @Override
            public void run() {
                try {
                    hw.linearSlideDriveMotor.setPower(-0.75);
                    Thread.sleep(1000);
                    hw.linearSlideDriveMotor.setPower(0);
                } catch (InterruptedException e) {

                }
            }
        };
        Runnable motorDrop = new Runnable() {
            @Override
            public void run() {
                try {
                    hw.linearSlideDriveMotor.setPower(0.75);
                    Thread.sleep(750);
                    hw.linearSlideDriveMotor.setPower(0);
                } catch (InterruptedException e) {

                }
            }
        };

        Thread liftGlyph = new Thread(motorLift);
        Thread dropGlpyh = new Thread(motorDrop);

        hw.altClawTurn.setPosition(0.5);  // center
        hw.upperLeft.setPosition(0.05);
        hw.upperRight.setPosition(0.90);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        resetFlickers();

        vuforia.close();
        detector = new CryptoboxDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, red ? TeamColor.RED : TeamColor.BLUE);
        detector.enable();
        hw.upperLeft.setPosition(0.19);
        hw.upperRight.setPosition(0.77);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Direction vumark = null;
            switch (lastKnownVumark) {
                case LEFT:
                    vumark = Direction.LEFT;
                    break;
                case RIGHT:
                    vumark = Direction.RIGHT;
                    break;
                case CENTER:
                    vumark = Direction.FORWARD;
                    break;
            }

            //jewelArms(!red);

            if (red) {
                // hw.phoneServo2.setPosition(0.26); // 0.64
            } else {
                hw.phoneServo1.setPosition(0.27);  // 0.27
            }

            hw.linearSlideDriveMotor.setPower(-0.75);
            sleep(1200);
            hw.linearSlideDriveMotor.setPower(0);

            // Open altClaw
            hw.altClawLeft.setPosition(0.29);  // 0.225
            hw.altClawRight.setPosition(0.65);  // 0.727

<<<<<<< HEAD
            //sleep(1000);

            //flick(!red, red);
            hw.linearSlideDriveMotor.setPower(0.75);
            sleep(1200);
            hw.linearSlideDriveMotor.setPower(0);
=======
            flick(!red, order);
            sleep(500);
>>>>>>> d8375ccf5fc20aad490eb4387f4abd104ef64140

            //dropGlpyh.start();

<<<<<<< HEAD
            //sleep(1000);
=======
            sleep(750);
>>>>>>> d8375ccf5fc20aad490eb4387f4abd104ef64140

            //resetJewelArms();
            //resetFlickers();

            // Close altClaw
            hw.altClawLeft.setPosition(0.67);  //0.610
            hw.altClawRight.setPosition(0.30);  //0.276
            sleep(500);
<<<<<<< HEAD

            hw.linearSlideDriveMotor.setPower(-0.75);
            sleep(1200);
            hw.linearSlideDriveMotor.setPower(0);

            //liftGlyph.start();
            //sleep(1200);

            AutoMove(0.25, 0, 1310);
            drivetrain.GyroTurn(0.15, 90);

=======
            liftGlyph.start();
            sleep(1000);
>>>>>>> d8375ccf5fc20aad490eb4387f4abd104ef64140

            if (close) {
                if (red) {
                    switch (vumark) {
                        case LEFT:
                            AutoMove(0.25, 0, 1170);
                            break;
                        case FORWARD:
                            AutoMove(0.25, 0, 1502);
                            break;
                        case RIGHT:
                            AutoMove(0.25, 0, 1834);
                            break;
                    }
                    drivetrain.GyroTurn(0.15, -90);
                } else {
                    switch (vumark) {
                        case LEFT:
                            AutoMove(0.25, 0, 1223);
                            break;
                        case FORWARD:
                            AutoMove(0.25, 0, 1567);
                            break;
                        case RIGHT:
                            AutoMove(0.25, 0, 1909);
                            break;
                    }
                    drivetrain.GyroTurn(0.15, 90);
                }
            } else {
                AutoMove(0.25, 0, 1050);

                if (red) {
                    /*moveHorizontal(0.15, 0.0003, 300, Direction.LEFT);
                    visionCrypto(0.1, 0.0003, Direction.LEFT, vumark);*/
                    switch (vumark) {
                        case LEFT:
                            moveHorizontal(0.1, 0.0003, 1095, Direction.LEFT);
                            break;
                        case FORWARD:
                            moveHorizontal(0.1, 0.0003, 655, Direction.LEFT);
                            break;
                        case RIGHT:
                            moveHorizontal(0.1, 0.0003, 300, Direction.LEFT);
                            break;
                    }
                } else {
                    switch (vumark) {
                        case LEFT:
                            moveHorizontal(0.1, 0.0003, 200, Direction.RIGHT);
                            break;
                        case FORWARD:
                            moveHorizontal(0.1, 0.0003, 635, Direction.RIGHT);
                            break;
                        case RIGHT:
                            moveHorizontal(0.1, 0.0003, 1075, Direction.RIGHT);
                            break;
                    }
                }
            }


            AutoMove(0.25, 0, 275);
            AutoMove(-0.25, 0, 25);

            hw.linearSlideDriveMotor.setPower(0.75);
            sleep(500);
            hw.linearSlideDriveMotor.setPower(0);

            release();
            AutoMove(-0.5, 0, 200);
            if (vumark != Direction.FORWARD) {
                AutoMove(0.2, vumark == Direction.RIGHT ? -90 : 90, 175);
            }

            hw.altClawLeft.setPosition(0.88);
            hw.altClawRight.setPosition(0.12);

            if (close) {
                if (red) {
                    drivetrain.GyroTurn(0.4, 90);
                } else {
                    drivetrain.GyroTurn(0.4, -90);
                }
            } else {
                if (red) {
                    drivetrain.GyroTurn(0.4, -180);
                } else {
                    drivetrain.GyroTurn(0.4, 180);
                }
            }
            AutoMoveByTime(-0.8, 0, 370, 2000);
            AutoMove(0.8, 0, 100);

            release();
            hw.upperLeft.setPosition(0.05);
            hw.upperRight.setPosition(0.90);

            requestOpModeStop();
        }

        hw.imu.stopAccelerationIntegration();

    }

    public void AutoMove(double speed, double angle, int counts) {
        int initialForward = hw.forwardLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.MoveAngle(speed, angle, 0);

        while (true) {
            int differenceForward = Math.abs(hw.forwardLeft.getCurrentPosition() - initialForward);
            int differenceBackward = Math.abs(hw.backLeft.getCurrentPosition() - initialBackward);
            telemetry.addData("d1", differenceForward);
            telemetry.addData("d2", differenceBackward);
            telemetry.update();
            if ((differenceBackward + differenceForward) / 2 > counts) {
                drivetrain.stop();
                break;
            }
        }
    }

    public void AutoMoveByTime(double speed, double angle, int counts, double timeout) {
        double start = runtime.milliseconds();
        int initialForward = hw.forwardLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive() && runtime.milliseconds() - start < timeout) {
            int differenceForward = Math.abs(hw.forwardLeft.getCurrentPosition() - initialForward);
            int differenceBackward = Math.abs(hw.backLeft.getCurrentPosition() - initialBackward);
            telemetry.addData("d1", differenceForward);
            telemetry.addData("d2", differenceBackward);
            telemetry.update();
            if ((differenceBackward + differenceForward) / 2 > counts) {
                drivetrain.stop();
                break;
            }
        }
    }

    public void flick(boolean left, JewelDetector.JewelOrder order) {

        if (left) {
            if (order == JewelDetector.JewelOrder.RED_BLUE) {
                hw.leftFlick.setPosition(1); // Flick back
            } else {
                hw.leftFlick.setPosition(0); // Flick forward
            }
        } else {
            if (order == JewelDetector.JewelOrder.RED_BLUE) {
                // Flick back
                hw.rightFlick.setPosition(0);
            } else {
                // Flick forward
                hw.rightFlick.setPosition(1);
            }
        }
    }

    void resetFlickers() {
        hw.leftFlick.setPosition(0.42);
        hw.rightFlick.setPosition(0.50);
    }

    class Encoders {
        int backLeft;
        int backRight;
        int forwardLeft;
        int forwardRight;

        public Encoders() {

        }

        public Encoders(RobotHardware robot) {
            this.backLeft = robot.backLeft.getCurrentPosition();
            this.backRight = robot.backRight.getCurrentPosition();
            this.forwardLeft = robot.forwardLeft.getCurrentPosition();
            this.forwardRight = robot.forwardRight.getCurrentPosition();
        }

        public void set(FourWheelMecanumDrivetrain dt, RobotHardware hw) {
            dt.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.backLeft.setTargetPosition(backLeft);
            hw.backRight.setTargetPosition(backRight);
            hw.forwardLeft.setTargetPosition(forwardLeft);
            hw.forwardRight.setTargetPosition(forwardRight);
        }

        public Encoders average(Encoders other) {
            Encoders e = new Encoders();
            e.backLeft = (this.backLeft + other.backLeft) / 2;
            e.backRight = (this.backRight + other.backRight) / 2;
            e.forwardLeft = (this.forwardLeft + other.forwardLeft) / 2;
            e.forwardRight = (this.forwardRight + other.forwardRight) / 2;
            return e;
        }

        public void applyOffset(Direction direction, double amount) {
            if (direction == Direction.FORWARD) {
                return;
            }
            if (direction == Direction.LEFT) {
                forwardLeft -= amount;
                forwardRight += amount;
                backLeft += amount;
                backRight -= amount;
            }
            if (direction == Direction.RIGHT) {
                forwardLeft += amount;
                forwardRight -= amount;
                backLeft -= amount;
                backRight += amount;
            }
        }

        public int totalDiff(Encoders a) {
            return Math.abs(a.backLeft - backLeft) + Math.abs(a.backRight - backRight) + Math.abs(a.forwardLeft - forwardLeft) + Math.abs(a.forwardRight - forwardRight);
        }
    }

    private Direction opposite(Direction input) {
        if (input == Direction.BACKWARD) {
            return Direction.FORWARD;
        }
        if (input == Direction.FORWARD) {
            return Direction.BACKWARD;
        }
        if (input == Direction.LEFT) {
            return Direction.RIGHT;
        }
        if (input == Direction.RIGHT) {
            return Direction.LEFT;
        }
        return null;
    }

    private void moveHorizontal(double speed, double p, double amount, Direction direction) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();

        while (opModeIsActive()) {
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

            if (avg >= amount) {
                break;
            }
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
            switch (desired) {
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
        } else {
            switch (desired) {
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
            return;
        } else {
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
                    } else {
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

    private void lightCrypto(double speed, double p, Direction direction, Direction desired, boolean blue, int offset) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();
        int count = 0;
        double firstTime = 0;
        double adjustedOffset = offset;
        if (opposite(direction) == desired) {
            adjustedOffset += 25;
            RobotLog.i("opp dir");
        }

        Encoders first = null;
        Encoders second = null;
        Encoders middle = null;

        double dominantColor;
        double secondaryColor;
        double multiplier;

        while (opModeIsActive()) {
            double timeElapsed = runtime.milliseconds() - firstTime;
            if (blue) {
                dominantColor = hw.bottom_color.blue();
                secondaryColor = hw.bottom_color.red();
                multiplier = 1.25;
            } else {
                dominantColor = hw.bottom_color.red();
                secondaryColor = hw.bottom_color.blue();
                multiplier = 1.25;
            }

            if (dominantColor > (secondaryColor * multiplier)) {

                first = new Encoders(hw);
                while (opModeIsActive() && dominantColor > (secondaryColor * multiplier)) {
                    if (blue) {
                        dominantColor = hw.bottom_color.blue();
                        secondaryColor = hw.bottom_color.red();
                    } else {
                        dominantColor = hw.bottom_color.red();
                        secondaryColor = hw.bottom_color.blue();
                    }
                }
                while (opModeIsActive() && !(dominantColor > (secondaryColor * multiplier))) {
                    if (blue) {
                        dominantColor = hw.bottom_color.blue();
                        secondaryColor = hw.bottom_color.red();
                    } else {
                        dominantColor = hw.bottom_color.red();
                        secondaryColor = hw.bottom_color.blue();
                    }
                }
                while (opModeIsActive() && (dominantColor > (secondaryColor * multiplier))) {
                    if (blue) {
                        dominantColor = hw.bottom_color.blue();
                        secondaryColor = hw.bottom_color.red();
                        multiplier = 1.25;
                    } else {
                        dominantColor = hw.bottom_color.red();
                        secondaryColor = hw.bottom_color.blue();
                        multiplier = 1.25;
                    }
                }
                second = new Encoders(hw);
                middle = first.average(second);
                middle.applyOffset(desired, adjustedOffset);
                middle.set(drivetrain, hw);
                drivetrain.setPowerAll(speed);
                break;

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
        double start = runtime.milliseconds();
        double timeout = 8000;
        Encoders e;
        e = new Encoders(hw);
        Encoders f = new Encoders(hw);
        int finaloffset = 150;
        double diff;
        boolean moving = true;
        double previousMs = runtime.milliseconds();
        while (opModeIsActive() && runtime.milliseconds() - start < timeout) {
            e = new Encoders(hw);
            double difference = e.totalDiff(middle);
            double avg = difference / 4;
            if (avg < 30) {
                break;
            }
        }

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.stop();
    }


    private void crypto(int ticks, double speed, double p, Direction direction) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();
        int currentTicks = 0;
        while (opModeIsActive() && currentTicks < ticks) {
            double distance = hw.ultrasonic.distance();
            if (distance < 5) {

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
        drivetrain.stop();
    }

    private void release() {
        hw.altClawLeft.setPosition(0.29);
        hw.altClawRight.setPosition(0.65);
    }

    private void jewelArms(boolean left) {
        if (left) {
            hw.jewelArm1.setPosition(0.76);  // 0.511
        } else {
            hw.jewelArm2.setPosition(0.34); // 0.349
        }
    }

    private void resetJewelArms() {
        hw.jewelArm1.setPosition(0.18);
        hw.jewelArm2.setPosition(0.95);
    }

    private void prestart(boolean red, boolean close) {
        int times = 0;
        while (times < 5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                if (lastKnownVumark == vuMark) {
                    times++;
                } else {
                    lastKnownVumark = vuMark;
                    times = 0;
                }
            } else {
                times = 0;
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.addData("Red", red);
            telemetry.addData("Close", close);
            telemetry.addData("Reliability check", times + "/5");
            telemetry.update();
            sleep(50);
        }
    }

    @Override
    public void waitForStart() {
        Image rgb = null;
        Mat tmp = new Mat();
        Mat tmpGray = new Mat();
        vuforia.setFrameQueueCapacity(10);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        while (!isStarted()) {
            try {
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                long numImages = frame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgb = frame.getImage(i);
                        break;
                    }
                }
                if (rgb != null) {
                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(rgb.getPixels());
                    tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC3);

                    Utils.bitmapToMat(bm, tmp);
                    Imgproc.cvtColor(tmp, tmpGray, Imgproc.COLOR_RGB2GRAY);
                    jewels.processFrame(tmp, tmpGray);
                    JewelDetector.JewelOrder order = jewels.getCurrentOrder();
                    resultsBuffer.add(new Pair<>(RelicRecoveryVuMark.from(relicTemplate), order));
                    if (resultsBuffer.size() > maxBufferResults) {
                        resultsBuffer.removeFirst();
                    }
                    Results results = analyzeBuffer();
                    telemetry.addData("Vision", results.toString());
                    telemetry.update();
                    lastKnownVumark = results.mostCommonVuMark;
                    order = results.mostCommonOrder;
                    frame.close();
                    Thread.sleep(50);
                }

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    class Results {
        public Map<RelicRecoveryVuMark, Integer> vumarkOccurences = new HashMap<>();
        public Map<JewelDetector.JewelOrder, Integer> jewelOccurences = new HashMap<>();
        RelicRecoveryVuMark mostCommonVuMark = RelicRecoveryVuMark.UNKNOWN;
        int vF;
        JewelDetector.JewelOrder mostCommonOrder = JewelDetector.JewelOrder.UNKNOWN;
        int oF;

        public Results() {
            vumarkOccurences.put(RelicRecoveryVuMark.LEFT, 0);
            vumarkOccurences.put(RelicRecoveryVuMark.CENTER, 0);
            vumarkOccurences.put(RelicRecoveryVuMark.RIGHT, 0);
            vumarkOccurences.put(RelicRecoveryVuMark.UNKNOWN, 0);

            jewelOccurences.put(JewelDetector.JewelOrder.BLUE_RED, 0);
            jewelOccurences.put(JewelDetector.JewelOrder.RED_BLUE, 0);
            jewelOccurences.put(JewelDetector.JewelOrder.UNKNOWN, 0);
        }

        @Override
        public String toString() {
            return "Most likely VuMark: " + mostCommonVuMark.toString() + "\n" +
                    "Certainty: " + (vF / maxBufferResults) * 100 + "%" + "\n" +
                    "Most likely Jewel Order: " + mostCommonOrder + "\n" +
                    "Certainty: " + (oF / maxBufferResults) * 100 + "%";
        }
    }

    public Results analyzeBuffer() {
        Results results = new Results();
        for (Pair<RelicRecoveryVuMark, JewelDetector.JewelOrder> pair : resultsBuffer) {
            RelicRecoveryVuMark vm = pair.first;
            JewelDetector.JewelOrder order = pair.second;

            results.vumarkOccurences.put(vm, results.vumarkOccurences.get(vm) + 1);
            results.jewelOccurences.put(order, results.jewelOccurences.get(order) + 1);
        }

        int maxVuMarkFrequency = Collections.max(results.vumarkOccurences.values());
        for (Map.Entry<RelicRecoveryVuMark, Integer> vuMarkIntegerEntry : results.vumarkOccurences.entrySet()) {
            if (vuMarkIntegerEntry.getValue() == maxVuMarkFrequency) {
                results.mostCommonVuMark = vuMarkIntegerEntry.getKey();
            }
        }

        results.vF = maxVuMarkFrequency;

        int maxOrderFrequency = Collections.max(results.jewelOccurences.values());
        for (Map.Entry<JewelDetector.JewelOrder, Integer> jewelIntegerEntry : results.jewelOccurences.entrySet()) {
            if (jewelIntegerEntry.getValue() == maxOrderFrequency) {
                results.mostCommonOrder = jewelIntegerEntry.getKey();
            }
        }

        results.oF = maxOrderFrequency;
        return results;
    }

}