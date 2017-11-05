package org.firstinspires.ftc.teamcode.Shared;

/**
 * Created by Joshua on 9/9/2017.
 */

public class FourWheelMecanumDrivetrain implements MecanumDrivetrain {
    RobotHardware rw = RobotHardware.GetSingleton();

    final double speedMultiplier = 0.75;
    final double rotSpeed = 0.5;
    final double speedThreshold = 0.05;

    @Override
    public void MoveCardinal(Direction direction, float speed) {
        switch (direction) {
            case FORWARD:
                rw.forwardRight.setPower(speed);
                rw.forwardLeft.setPower(speed);
                rw.backRight.setPower(speed);
                rw.backLeft.setPower(speed);
                break;
            case BACKWARD:
                rw.forwardRight.setPower(-speed);
                rw.forwardLeft.setPower(-speed);
                rw.backRight.setPower(-speed);
                rw.backLeft.setPower(-speed);
                break;
            case LEFT:
                rw.forwardRight.setPower(speed);
                rw.forwardLeft.setPower(-speed);
                rw.backRight.setPower(-speed);
                rw.backLeft.setPower(speed);
                break;
            case RIGHT:
                rw.forwardRight.setPower(-speed);
                rw.forwardLeft.setPower(speed);
                rw.backRight.setPower(speed);
                rw.backLeft.setPower(-speed);
                break;

        }
    }

    public void Rotate(boolean clockwise, double speed) {
        if (clockwise) {
            rw.forwardRight.setPower(speed);
            rw.forwardLeft.setPower(-speed);
            rw.backRight.setPower(speed);
            rw.backLeft.setPower(-speed);
        }
        else {
            rw.forwardRight.setPower(-speed);
            rw.forwardLeft.setPower(speed);
            rw.backRight.setPower(-speed);
            rw.backLeft.setPower(speed);
        }
    }

    @Override
    public void MoveAngle(double speed, double angle, double turn) {
        double vRot = turn;

        double desiredAngle = (angle) + Math.PI / 4;
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

        if (Math.abs(rightBackward) < speedThreshold) {
            rightBackward = 0;
        }
        if (Math.abs(rightForward) < speedThreshold) {
            rightForward = 0;
        }
        if (Math.abs(leftBackward) < speedThreshold) {
            leftBackward = 0;
        }
        if (Math.abs(leftForward) < speedThreshold) {
            leftForward = 0;
        }
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
