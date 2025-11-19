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
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
Port Configs
Control Hub
Motors
0 - fr
1 - fl
2 - br
3 - bl
Servos - none
Expansion Hub
Motors
0 - empty
1 - transfer
2 - launch
3 - intake
Servos
0 - hood
4 - cycle
 */
public class Robot {
    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotorEx intake,launch;

    public final Servo cycle,transfer;
    public static LinearOpMode opMode;
    public static  double[] cyclePos = new double[3];
    public static double[] shootPos = new double[3];
    private int var = 0;
    private final Limelight3A limelight;
    private final double transferPower = 0.9;
    public Robot(LinearOpMode opMode) {
        //TODO
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
        bl.setDirection(Direction.FORWARD);
        br.setDirection(Direction.REVERSE);

        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initializing other motors

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        cycle =  hardwareMap.get(Servo.class,"cycle");

        intake.setMode(RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        cyclePos[0] = 0;
        cyclePos[1] = 1/3.0;
        cyclePos[2] = 2/3.0;

        shootPos[0] = 1/6.0;
        shootPos[1] = 1/2.0;
        shootPos[2] = 5/6.0;

        // Outtake config

        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        launch.setDirection(Direction.REVERSE);

        transfer = hardwareMap.get(Servo.class, "transfer");

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
    // Add distance to hood angle mapping
    public void outtake(char color){

        double Kv = 0.00039;
        double Kp = 0.001;

        double goalHeight = 29.5;
        double limelightHeight = 10.5;
        double angle = 0;
        double x = 0;
        int targetId = 0;
        if(color == 'r'){
            targetId = 24;
        }
        else{
            targetId = 20;
        }
        cycle.setPosition(shootPos[var]);
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            if (id == targetId) {
                angle = Math.toRadians(y);
                break;
            }
        }
        x = (goalHeight - limelightHeight) / Math.tan(angle);
        double targetVelo = 394.56267*x*Math.pow((0.25183*x-0.17955),-0.5);
        double ff = Kv * targetVelo;
        double currentVelo = launch.getVelocity();
        double p = Kp * (targetVelo - currentVelo);
        double power = Range.clip(ff + p, -0.2, 1.0);
        transfer.setPosition(1);
        launch.setPower(power);
        cycle.setPosition(shootPos[var]);
    }

    public void stopOuttake(int reset){
        if(reset == 1)
            {var += 1;
        setCycle(var);}
        transfer.setPosition(0);
        launch.setPower(0);
    }
    public void setCycle(int pos){
        var = pos % 3;
        cycle.setPosition(cyclePos[var]);
    }
    public void setMotor(int motor, double power){
        if(motor == 0) {
            fl.setPower(power);
        }
        else if(motor == 1) {
            fr.setPower(power);
        }
        else if(motor == 2) {
            bl.setPower(power);
        }
        else if(motor == 3) {
            br.setPower(power);
        }
    }
    public void setLaunch(double power){
        launch.setPower(power);
    }
    public double getLaunchVelo(){
        return launch.getVelocity();
    }
}
