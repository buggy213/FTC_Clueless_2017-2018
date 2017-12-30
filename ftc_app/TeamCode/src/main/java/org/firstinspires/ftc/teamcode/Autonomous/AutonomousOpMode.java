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

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;
import org.firstinspires.ftc.teamcode.Shared.VL53L0X;
import org.opencv.core.Mat;


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

    // Used to access robot hardware
    RobotHardware hw;

    FourWheelMecanumDrivetrain drivetrain;

    // Stores instance of Vuforia Localizer
    VuforiaLocalizer vuforia;

    // Instance of relic trackable
    VuforiaTrackable relicTemplate;

    RelicRecoveryVuMark lastKnownVumark = RelicRecoveryVuMark.UNKNOWN;

    double distanceThreshold;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = RobotHardware.GetSingleton(hardwareMap);
        // Match     parameters = MatchParameters.loadParameters(FtcRobotControllerActivity.matchParameterData);
        drivetrain = new FourWheelMecanumDrivetrain();
        hw.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hw.phoneServo1.setPosition(0.04);
        // MatchParameters parameters = MatchParameters.loadParameters(FtcRobotControllerActivity.matchParameterData);
        if (hw.imu == null) {
            hw.ReinitializeIMU();
        }
        hw.imu.startAccelerationIntegration(new Position(), new Velocity(), 16);


        //region Vuforia
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = "ASK0CtX/////AAAAGbyNvvxXMUrohVJwXgMBW5Iqmz//UVeASb2KC//wHTXPuoN/gOM0vbw91nX++j+iS98pzeEfO+p9jpijt7j6VgQZlFTO9K2HjwAvTmG5M6CYglrh0B3kfA/nZx/NSyyxWIRe7Q03DeNDH50ZnSJ3I4FkyD7AbcTbJHg3LjL72N6/Lfm5biUbhOPoeQb1a8qUaqp1Il340pGFEvIEH8s7nhqHAga3TSdvM7yWxqRtZ3Bv2yEmIFMIWuBdEV6ahooWDsnnRmSn33bQVRD+KTNNocJWkwQ1pDG/8XBzswiICKBvcCvzklZhV/TeoJDk6tagk8sMcYxBdzMtyBlYVA9y3iJRyuzu+UMfswmSlA6CnmM/";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        //endregion

        Runnable motorLift = new Runnable() {
            @Override
            public void run() {
                try {
                    Thread.sleep(150);
                    hw.linearSlideDriveMotor.setPower(0.5);
                    Thread.sleep(1500);
                    hw.linearSlideDriveMotor.setPower(0);
                } catch (InterruptedException e) {

                }
            }
        };
        Runnable motorDrop = new Runnable() {
            @Override
            public void run() {
                try {
                    hw.linearSlideDriveMotor.setPower(-0.5);
                    Thread.sleep(500);
                    hw.linearSlideDriveMotor.setPower(0);
                } catch (InterruptedException e) {

                }
            }
        };

        Thread liftGlyph = new Thread(motorLift);
        Thread dropGlpyh = new Thread(motorDrop);

        hw.right_color.enableLed(true);
        // telemetry.addData("Blue", parameters.getInt("blue_threshold"));
        // Wait for the game to start (driver presses PLAY)
        this.waitForStart();
        runtime.reset();
        lightCrypto(0, 0.1, 0.0003, Direction.RIGHT);
        //crypto(2, 0.08, 0.0003, Direction.RIGHT);
        /*
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            String mode = parameters.get("start");
            boolean close = mode.contains("CLOSE");
            boolean red = mode.contains("RED");

            grab();
            liftGlyph.start();
            hw.altClawTurn.setPosition(0.5);  // center

            if (red) {
                AutoMove(0.2, 0, 65);
                Thread.sleep(1000);
                jewelArms(false);
                Thread.sleep(2500);
                int redVal = hw.right_color.red();
                int blue = hw.right_color.blue();
                boolean detectRed;
                detectRed = redVal > blue;

                if (detectRed) {
                    AutoMove(-0.2, 0, 90);
                    resetJewelArms();
                    Thread.sleep(600);
                    if (lastKnownVumark == RelicRecoveryVuMark.RIGHT) {
                        AutoMove(0.2, 0, 1350);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.CENTER) {
                        AutoMove(0.2, 0, 1700);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.LEFT) {
                        AutoMove(0.2, 0, 2050);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.UNKNOWN) {
                        AutoMove(0.2, 0, 1700);
                    }
                } else {
                    AutoMove(0.2, 0, 90);
                    resetJewelArms();
                    Thread.sleep(600);
                    if (lastKnownVumark == RelicRecoveryVuMark.RIGHT) {
                        AutoMove(0.2, 0, 1000);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.CENTER) {
                        AutoMove(0.2, 0, 1350);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.LEFT) {
                        AutoMove(0.2, 0, 1600);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.UNKNOWN) {
                        AutoMove(0.2, 0, 1350);
                    }
                }
            } else {
                AutoMove(0.2, 0, 65);
                Thread.sleep(1000);
                jewelArms(true);
                Thread.sleep(2500);
                int redVal = hw.left_color.red();
                int blue = hw.left_color.blue();
                boolean detectRed;
                detectRed = redVal > blue;
                RobotLog.i("USER Red value" + redVal);
                RobotLog.i("USER Blue value" + blue);
                RobotLog.i("USER Red detected" + detectRed);
                if (detectRed) {
                    AutoMove(0.2, 0, 90);
                    resetJewelArms();
                    Thread.sleep(600);
                    if (lastKnownVumark == RelicRecoveryVuMark.LEFT) {
                        AutoMove(0.2, 0, 1000);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.CENTER) {
                        AutoMove(0.2, 0, 1350);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.RIGHT) {
                        AutoMove(0.2, 0, 1600);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.UNKNOWN) {
                        AutoMove(0.2, 0, 1350);
                    }
                } else {
                    AutoMove(-0.2, 0, 90);
                    resetJewelArms();
                    Thread.sleep(600);
                    if (lastKnownVumark == RelicRecoveryVuMark.LEFT) {
                        AutoMove(0.2, 0, 1350);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.CENTER) {
                        AutoMove(0.2, 0, 1700);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.RIGHT) {
                        AutoMove(0.2, 0, 2050);
                    }
                    if (lastKnownVumark == RelicRecoveryVuMark.UNKNOWN) {
                        AutoMove(0.2, 0, 1700);
                    }
                }
            }
            if (close) {
                if (red) {
                    // Angle is counterclockwise
                    drivetrain.GyroTurn(0.15, -88.5);
                } else {
                    drivetrain.GyroTurn(0.15, 88.5);
                }
            } else {

            }

            dropGlpyh.start();
            drivetrain.AutoMove(0.2, 0, 1.5);
            AutoMove(-0.2, 0, 50);
            release();
            AutoMove(-0.2, 0, 150);

            /*if (close) {
                AutoMove(-0.2, 0, 300);
                drivetrain.OneEighty(90, 0.2);
            }*/


            /*requestOpModeStop();
        }*/

        hw.imu.stopAccelerationIntegration();

    }

    public void AutoMove(double speed, double angle, int counts) {
        int initialForward = hw.forwardLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive()) {
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

    private void grab() {
        hw.altClawLeft.setPosition(0.610);
        hw.altClawRight.setPosition(0.276);
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

        public void compensate(RobotHardware hw, double speed, Encoders target, Direction direction, double p) {
            if (direction == Direction.LEFT) {
                //double backLeftComp = (1) * p;
                //double backRightComp = (avg - backRightDiff) * p;
                //double forwardLeftComp = (avg - forwardLeftDiff) * p;
                //double forwardRightComp = (avg - forwardRightDiff) * p;
            }
            else {

            }
        }
    }

   /* private Direction opposite(Direction input) {
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
    } */

    private void lightCrypto(double blueThreshold, double speed, double p, Direction direction) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();
        int count = 0;
        double firstTime = 0;
        int offset = 100;
        
        Encoders first = null;
        Encoders second = null;
        Encoders middle = null;
        while (opModeIsActive()) {
            double timeElapsed = runtime.milliseconds() - firstTime;
            if (hw.bottom_color.blue() > (hw.bottom_color.red() * 2) && (timeElapsed > 2500 || count == 0)) {

                first = new Encoders(hw);
                while (opModeIsActive() && hw.bottom_color.blue() > (hw.bottom_color.red() * 2)) {

                }

                while (opModeIsActive() && !(hw.bottom_color.blue() > (hw.bottom_color.red() * 2))){

                }
                second = new Encoders(hw);
                middle = first.average(second);
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
        double timeout = 3000;
        while (opModeIsActive() && runtime.milliseconds() - start < timeout) {
            Encoders e = new Encoders(hw);

        }
        drivetrain.stop();
    }
    /*private void crypto(int ticks, double speed, double p, Direction direction) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();
        int currentTicks = 0;
        while(opModeIsActive() && currentTicks < ticks) {
            double distance = 0;
            try {
                hw.distanceSensor.startRanging(VL53L0X.VL53L0X_BEST_ACCURACY_MODE);
            }
            catch(Exception e) {
                RobotLog.e(e.toString());
            }
            try {
                distance = hw.distanceSensor.getDistance();
            }
            catch (Exception e) {
                RobotLog.e(e.toString());
            }

            if (distance < 150 && distance != 0 && distance != 20) {
                currentTicks++;
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
    }*/

    private void release() {
        hw.altClawLeft.setPosition(0.225);
        hw.altClawRight.setPosition(0.727);
    }

    private void jewelArms(boolean left) {
        if (left) {
            hw.jewelArm1.setPosition(0.479);
        } else {
            hw.jewelArm2.setPosition(0.446);
        }
    }

    private void resetJewelArms() {
        hw.jewelArm1.setPosition(0);
        hw.jewelArm2.setPosition(1);
    }

    private void prestart() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            lastKnownVumark = vuMark;
        } else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
    }

    public void waitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            prestart();
        }
    }
}