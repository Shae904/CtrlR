package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Outtake {
    public final DcMotorEx launch;
    public Outtake(LinearOpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(RunMode.RUN_WITHOUT_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    }
}
