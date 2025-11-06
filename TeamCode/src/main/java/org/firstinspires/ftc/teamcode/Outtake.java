package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Outtake {
    public final DcMotorEx launch;
    private final double launchVelo = 14000;
    private final double transferVelo = 10000;
    public final DcMotorEx transfer;
    public final Servo hood;
    public Outtake(LinearOpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        hood = hardwareMap.get(Servo.class,"hood");
    }
    public void outtake(){
        transfer.setVelocity(transferVelo);
        launch.setVelocity(launchVelo);
    }
    public void setHood(double hoodPos){
        hood.setPosition(hoodPos);
    }
}
