package org.firstinspires.ftc.teamcode.Shared;

/**
 * Created by Joshua on 9/9/2017.
 */

public class FourWheelMecanumDrivetrain implements MecanumDrivetrain {
    RobotHardware rw = RobotHardware.GetSingleton();

    final double speedMultiplier = 0.75;
    final double rotSpeed = 0.5;

    @Override
    public void MoveCardinal(Direction direction, float speed) {

    }

    @Override
    public void MoveAngle(double speed, double angle, double turn) {
        double vRot = rotSpeed * turn;

        double desiredAngle = (angle) - Math.PI / 4;
        if (desiredAngle < 0) {
            desiredAngle = desiredAngle + 2 * Math.PI;
        }
        if (desiredAngle >= 2 * Math.PI) {
            desiredAngle = desiredAngle % (2 * Math.PI);
        }

        double intermediateSin = Math.sin(desiredAngle);
        double intermediateCos = Math.cos(desiredAngle);

        double leftForward = speed * (intermediateSin) + vRot;
        double leftBackward = speed * (intermediateCos) + vRot;
        double rightForward = speed * (intermediateCos) - vRot;
        double rightBackward = speed * (intermediateSin) - vRot;

        rw.forwardRight.setPower(rightForward);
        rw.forwardLeft.setPower(leftForward);
        rw.backRight.setPower(rightBackward);
        rw.backLeft.setPower(leftBackward);
    }

    public void stop() {
        rw.forwardRight.setPower(0);
        rw.forwardLeft.setPower(0);
        rw.backRight.setPower(0);
        rw.backLeft.setPower(0);
    }
}
