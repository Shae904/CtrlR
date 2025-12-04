package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
Expansion Hub
Motors
2 - launch
3 - intake
Servos
0 - transfer
4 - cycle
IUC Bus 0
0 - zero
IUC Bus 1
0 - one
IUC Bus 2
0 - two
 */
@Configurable
public class Robot {
    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotorEx intake,launch;
    public static double transferOne = 0;
    public static double transferTwo = 0;
    public final ServoImplEx transfer,cycle;
    public static double transferMin = 0;
    public static double transferMax = 0.1;
    public static LinearOpMode opMode;
    public final ColorSensor zero,one,two;
    public static  double[] cyclePos = {0,0.32,0.59};
    public static double[] shootPos = {0.14,0.45,0.74};
    private int var = 0;
    private final Limelight3A limelight;

    public int purpleRThreshold = 128;
    public int purpleBThreshold = 128;
    public int greenGThreshold = 128;

    private final double transferPower = 0.9;
    public Robot(LinearOpMode opMode) {
        //TODO
        // Set bounds for hood and cycle servos
        Robot.opMode = opMode;
            HardwareMap hardwareMap = opMode.hardwareMap;
        // Drivetrain
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");

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

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        opMode.telemetry.addData("IMU Initialized", true);
        opMode.telemetry.update();

        // Initializing other motors
        cycle =  (ServoImplEx) hardwareMap.get(Servo.class,"cycle");

        intake = hardwareMap.get(DcMotorEx.class, "intake");



        intake.setMode(RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Outtake config

        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        launch.setDirection(Direction.REVERSE);

        transfer = (ServoImplEx) hardwareMap.get(Servo.class, "transfer");
        transfer.scaleRange(0.23,0.67);

        // Limelight config

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        // Color Sensor Config

        zero = hardwareMap.get(ColorSensor.class, "csfront");
        one = hardwareMap.get(ColorSensor.class, "csleft");
        two = hardwareMap.get(ColorSensor.class, "csright");




    }

    public Limelight3A getLimelight() {
        return limelight;
     }

    public double getHeading() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void setIntakePower(double intakePower){
        intake.setPower(intakePower);
    }
    //TODO
    // Add distance to hood angle mapping
    public void outtake(char color, double runtime){
        double cycleTime = 0.4; // TODO Tune
        double outTime = 0.8; // TODO Tune
        double Kv = 0.00039;
        double Kp = 0.001;
        double goalHeight = 29.5;
        double limelightHeight = 10.5;
        double angle = 0;
        double x;
        int targetId;
        if(color == 'r'){
            targetId = 24;
        }
        else{
            targetId = 20;
        }
//        cycle.setPosition(shootPos[var]);
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
        cycle.setPosition(shootPos[var]);
        launch.setPower(power);
        while(runtime >= cycleTime && runtime < cycleTime + outTime){
            transfer.setPosition(1);
        }
        while(runtime >= cycleTime + outTime && runtime < 2 * cycleTime + outTime) {
            transfer.setPosition(0);
            var += 1;
            setCycle(var);
        }
        while(runtime >= 2 * cycleTime + outTime && runtime < 2 * cycleTime + 2 * outTime){
            transfer.setPosition(1);
        }
        while(runtime >= 2 * cycleTime + 2 * outTime && runtime < 3 * cycleTime + 2 * outTime) {
            transfer.setPosition(0);
            var += 1;
            setCycle(var);
        }
        while(runtime >= 3 * cycleTime + 2 * outTime){
            transfer.setPosition(1);
        }
    }

    public void stopOuttake(int reset){
        if(reset == 1){
            var += 1;
            setCycle(var);
        }
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
    public int[] getColorReadings(){
        int[] out = {0,0,0};

        if(zero.red() >= purpleRThreshold && zero.blue() >= purpleBThreshold){
            out[0] = 1;
        }
        else if(zero.green() >= greenGThreshold){
            out[0] = -1;
        }

        if(one.red() >= purpleRThreshold && one.blue() >= purpleBThreshold){
            out[1] = 1;
        }
        else if(one.green() >= greenGThreshold){
            out[1] = -1;
        }

        if(two.red() >= purpleRThreshold && two.blue() >= purpleBThreshold){
            out[2] = 1;
        }
        else if(two.green() >= greenGThreshold){
            out[2] = -1;
        }
        return out;
    }

    public void setTransferOne() {
        transfer.setPosition(transferOne);
    }
    public void setTransferTwo(){
        transfer.setPosition(transferTwo);
    }

}
