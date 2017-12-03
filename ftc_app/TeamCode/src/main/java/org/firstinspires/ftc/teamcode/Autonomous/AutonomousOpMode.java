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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Autonomous", group="Linear Opmode")
public class AutonomousOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Used to access robot hardware
    RobotHardware hw = RobotHardware.GetSingleton(hardwareMap);

    FourWheelMecanumDrivetrain drivetrain;

    // Stores instance of Vuforia Localizer
    VuforiaLocalizer vuforia;

    // Instance of relic trackable
    VuforiaTrackable relicTemplate;

    RelicRecoveryVuMark lastKnownVumark = RelicRecoveryVuMark.UNKNOWN;

    double distanceThreshold;

    @Override
    public void runOpMode() throws InterruptedException{

        MatchParameters parameters = MatchParameters.loadParameters(FtcRobotControllerActivity.matchParameterData);

        hw.imu.startAccelerationIntegration(new Position(), new Velocity(), 50);

        drivetrain = new FourWheelMecanumDrivetrain();

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

        // Wait for the game to start (driver presses PLAY)
        this.waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            String mode = parameters.get("start");
            boolean close = mode.contains("CLOSE");
            boolean red = mode.contains("RED");

            if (red) {
                jewelArms(false);
                wait(300);
                int redVal = hw.right_color.red();
                int blue = hw.right_color.blue();
                boolean detectRed;
                boolean detectBlue;
                detectRed = redVal > parameters.getInt("red_threshold");

                detectBlue = blue > parameters.getInt("blue_threshold");

                if (detectBlue && !detectRed) {
                    drivetrain.turn(true, 0.2, 0.5);
                }
                else if (detectRed && !detectBlue) {
                    drivetrain.turn(false, 0.2, 0.5);
                }
                else {
                    // Well fuck. Guess you just keep going?
                }
            }
            else {
                jewelArms(true);
                wait(300);
                int redVal = hw.right_color.red();
                int blue = hw.right_color.blue();
                boolean detectRed;
                boolean detectBlue;
                detectRed = redVal > parameters.getInt("red_threshold");

                detectBlue = blue > parameters.getInt("blue_threshold");

                if (detectBlue && !detectRed) {
                    drivetrain.turn(false, 0.2, 0.5);
                }
                else if (detectRed && !detectBlue) {
                    drivetrain.turn(true, 0.2, 0.5);
                }
                else {
                    // Well fuck. Guess you just keep going?
                }
            }

            drivetrain.GyroTurn(0.2, 0);
            drivetrain.AutoMove(0.5, 0, 5000);

            if (close) {
                if (red) {
                    drivetrain.GyroTurn(0.2, 90);
                    moveCrypto(false);
                }
                else {
                    drivetrain.GyroTurn(0.2, -90);
                    moveCrypto(true);
                }
            }
            else {
                if(red) {
                    moveCrypto(false);
                }
                else {
                    moveCrypto(true);
                }
            }

            hw.imu.stopAccelerationIntegration();

            idle();
        }
    }

    public void moveCrypto(boolean fromLeft) {
        int index = 0;
        while (true) {
            drivetrain.MoveCardinal(fromLeft ? Direction.RIGHT : Direction.LEFT, 0.4f);
            if (hw.sensorDistance.getDistance(DistanceUnit.CM) < distanceThreshold) {
                index++;
            }
        }
    }



    // TODO add proper servo values

    private void jewelArms(boolean left) {
        if (left) {
            hw.jewelArm1.setPosition(0);
        } else {
            hw.jewelArm2.setPosition(0);
        }
    }
    private void resetJewelArms() {
        hw.jewelArm1.setPosition(0);
        hw.jewelArm2.setPosition(0);
    }

    private void prestart() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            lastKnownVumark = vuMark;
        }
        else {
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
