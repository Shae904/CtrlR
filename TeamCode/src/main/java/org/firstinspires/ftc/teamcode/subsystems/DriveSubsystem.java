package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private double strafeScale = 1.1;

    public DriveSubsystem(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        fl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        bl.setDirection(Direction.REVERSE);
        br.setDirection(Direction.FORWARD);

        fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    public void setStrafeScale(double scale) {
        this.strafeScale = scale;
    }

    public double getStrafeScale() {
        return strafeScale;
    }

    public void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void setMotor(int motorIndex, double power) {
        switch (motorIndex) {
            case 0:
                fl.setPower(power);
                break;
            case 1:
                fr.setPower(power);
                break;
            case 2:
                bl.setPower(power);
                break;
            case 3:
                br.setPower(power);
                break;
            default:
                break;
        }
    }

    public void driveRobotCentric(double y, double x, double rx) {
        driveRobotCentric(y, x, rx, true);
    }

    public void driveRobotCentric(double y, double x, double rx, boolean applyStrafeScale) {
        double xAdj = applyStrafeScale ? x * strafeScale : x;
        applyMecanum(y, xAdj, rx);
    }

    public void driveFieldCentric(IMU imu, double y, double x, double rx) {
        driveFieldCentric(imu, y, x, rx, true);
    }

    public void driveFieldCentric(IMU imu, double y, double x, double rx, boolean applyStrafeScale) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        if (applyStrafeScale) {
            rotX *= strafeScale;
        }

        applyMecanum(rotY, rotX, rx);
    }

    private void applyMecanum(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}
