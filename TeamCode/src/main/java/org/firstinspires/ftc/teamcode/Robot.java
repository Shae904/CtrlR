package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotor intake;
    public final DcMotor transfer;
    public final Servo cycle;
    private final Outtake out;
    private final double intakePower = 0.5;



    private final LinearOpMode opMode;
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        out = new Outtake(opMode);
        // Drivetrain
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");



        fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        bl.setDirection(Direction.REVERSE);
        br.setDirection(Direction.FORWARD);

        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initializing other motors
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
        cycle =  hardwareMap.get(Servo.class,"cycle");

        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    }

    public double getHeading() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDriveTrainPower(double frPow, double flPow, double brPow, double blPow) {
        fr.setPower(frPow);
        fl.setPower(flPow);
        br.setPower(brPow);
        bl.setPower(blPow);
    }
    public void shoot(){

    }
    public void intake(){

    }
    public void cycleCW(){
        double pos = cycle.getPosition();
        if(pos >= 2/3.0){
            cycle.setPosition(0);
        }
        else{
            cycle.setPosition(pos + 1/3.0);
        }
    }
    public void cycleCCW(){
        double pos = cycle.getPosition();
        if(pos < 0.1){
            cycle.setPosition(2/3.0);
        }
        else{
            cycle.setPosition(pos - 1/3.0);
        }
    }
}
