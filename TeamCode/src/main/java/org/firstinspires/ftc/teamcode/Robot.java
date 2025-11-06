package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotorEx intake,transfer,launch;
    public final Servo cycle;
    private final LinearOpMode opMode;
    private final double[] cyclePos = new double[3];
    private int var = 0;
    private Limelight3A limelight;
    public final Servo hood;
    private final double launchVelo = 14000;
    private final double transferVelo = 10000;
    public Robot(LinearOpMode opMode) {
        // TODO
        // Set bounds for hood and cycle servos
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
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

        intake = hardwareMap.get(DcMotorEx.class,"intake");

        cycle =  hardwareMap.get(Servo.class,"cycle");

        intake.setMode(RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        cyclePos[0] = 0;
        cyclePos[1] = 1/3.0;
        cyclePos[2] = 2/3.0;

        // Outtake config

        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        hood = hardwareMap.get(Servo.class,"hood");

        // Limelight config

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

    }

    public Limelight3A getLimelight() {
        return limelight;
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
    public void setIntakePower(double intakePower){
        intake.setPower(intakePower);
    }
    //TODO
    /*
        Add distance to hood angle mapping
     */
    public void outtake(char color){
        double goalHeight = 30.0;
        double limelightHeight = 12.5;
        double angle = 0;
        double distance = 0;
        int targetId = 0;
        if(color == 'r'){
            targetId = 24;
        }
        else{
            targetId = 20;
        }
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            if(id == targetId) {
                angle = Math.toRadians(y);
            }
        }
        distance = (goalHeight - limelightHeight) / Math.tan(angle);

        // Insert distance to hood conversion here

        transfer.setVelocity(transferVelo);
        launch.setVelocity(launchVelo);
    }
    public void cycleCW(){
        var += 1;
        if(var >= 3){
            var = 0;
        }
        cycle.setPosition(cyclePos[var]);
    }
    public void cycleCCW(){
        var -= 1;
        if(var <= -1){
            var = 2;
        }
        cycle.setPosition(cyclePos[var]);
    }
    public void setHood(double pos){
        hood.setPosition(pos);
    }
}
