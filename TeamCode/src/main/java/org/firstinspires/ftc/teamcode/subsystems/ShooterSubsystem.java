package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class ShooterSubsystem {
    private final DcMotorEx launch;
    private final ServoImplEx transfer;
    private final ServoImplEx cycle;
    private final Limelight3A limelight;

    private double distanceX = 128;

    public ShooterSubsystem(DcMotorEx launch, ServoImplEx transfer, ServoImplEx cycle, Limelight3A limelight) {
        this.launch = launch;
        this.transfer = transfer;
        this.cycle = cycle;
        this.limelight = limelight;

        launch.setMode(RunMode.RUN_WITHOUT_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        launch.setDirection(Direction.REVERSE);
    }

    public void setLaunchPower(double power) {
        launch.setPower(power);
    }

    public double getLaunchVelocity() {
        return launch.getVelocity();
    }

    public DcMotorEx getLaunchMotor() {
        return launch;
    }

    public void setCyclePosition(double position) {
        cycle.setPosition(position);
    }

    public void setTransferPosition(double position) {
        transfer.setPosition(position);
    }

    public double outtake(char color, double launchMult) {
        double Kv = 0.000379;
        double Kp = 0.001;

        double goalHeight = 30;
        double limelightHeight = 10;

        double angle = -1;
        int targetId = (color == 'r') ? 24 : 20;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetId) {
                        double y = fiducial.getTargetYDegrees();
                        angle = Math.toRadians(y + 21); // 21 = limelight mount tilt
                        break;
                    }
                }
            }
        }

        if (angle != -1) {
            distanceX = (goalHeight - limelightHeight) / Math.tan(angle) + 6; // +6 = limelight->shooter offset
        }

        double targetVelo = launchMult * distanceX * Math.pow(1.16825863336 * distanceX - 29, -0.5);
        double ff = Kv * targetVelo;

        double currentVelo = launch.getVelocity();
        double p = Kp * (targetVelo - currentVelo);

        double power = Range.clip(ff + p, -0.2, 1.0);
        launch.setPower(power);

        return targetVelo;
    }
}
