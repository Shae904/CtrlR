package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem {
    private final DcMotorEx motor;

    public IntakeSubsystem(DcMotorEx motor) {
        this.motor = motor;
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }

    public DcMotorEx getMotor() {
        return motor;
    }
}
