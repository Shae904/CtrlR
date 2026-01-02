package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Red Far")
public class RedFar extends LinearOpMode {
    public static Robot robot;
    public static ElapsedTime opModeTimer;

    public static double cycleTime = Robot.cycleTime;
    public static double outTime = Robot.outTime;
    public static double transferTime = Robot.transferTime;

    public static double[][] PoseCoords = {
            {87,9,90}, // Start
            {90,12,70}, // Shoot
            {83,84,0}, // Intake PPG Start
            {114,84,0}, // Intake PPG End
            {83,60,0}, // Intake PGP Start
            {114,60,0}, // Intake PGP End
            {96,35,0}, // Intake GPP Start
            {136,35,0}, // Intake GPP End
            {90,50,62} // PARK
    };
    public static Pose START,SHOOT,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END,PARK;
    public Pose[] Poses = {START,SHOOT,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END,PARK};
    public Follower follower;
    public static int pattern = 21;
    public Limelight3A limelight;
    public final Telemetry telemetry;
    private int shooting;
    public C920PanelsEOCV.C920Pipeline.SlotState[] colors;

    public enum PathState{PRELOAD,GPP,PARK,STOP}

    private PathState pathState = PathState.PRELOAD;

    public RedFar(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initializePoses(){
        for(int i = 0; i < Poses.length; i++) {
           Poses[i] = new Pose(PoseCoords[i][0],PoseCoords[i][1],Math.toRadians(PoseCoords[i][2]));
        }
    }

    public PathChain shootPreload,preIntakePPG,intakePPG,launchPPG,preIntakePGP,intakePGP,launchPGP,preIntakeGPP,intakeGPP,shootGPP,park;
    public void buildPaths(){
        initializePoses();
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(START,SHOOT))
                .setLinearHeadingInterpolation(START.getHeading(), SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();

        /*preIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,INTAKE_PPG_START))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), INTAKE_PPG_START.getHeading())
                .setTimeoutConstraint(500)
                .build();
        intakePPG = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PPG_START,INTAKE_PPG_END))
                .setConstantHeadingInterpolation(INTAKE_PPG_START.getHeading())
                .setTimeoutConstraint(500)
                .build();
        launchPPG = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PPG_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_PPG_START.getHeading(),SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();

        preIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,INTAKE_PGP_START))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), INTAKE_PGP_START.getHeading())
                .setTimeoutConstraint(500)
                .build();
        intakePGP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PGP_START,INTAKE_PGP_END))
                .setConstantHeadingInterpolation(INTAKE_PGP_START.getHeading())
                .setTimeoutConstraint(500)
                .build();
        launchPGP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PGP_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_PGP_START.getHeading(),SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();*/

        preIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,INTAKE_GPP_START))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), INTAKE_GPP_START.getHeading())
                .setTimeoutConstraint(500)
                .build();
        intakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_GPP_START,INTAKE_GPP_END))
                .setConstantHeadingInterpolation(INTAKE_GPP_START.getHeading())
                .setTimeoutConstraint(500)
                .build();
        shootGPP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_GPP_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_GPP_START.getHeading(),SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,PARK))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), PARK.getHeading())
                .setTimeoutConstraint(500)
                .build();
    }
    public void intakeThree(PathChain prePath, PathChain intakePath){
        follower.followPath(prePath);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        robot.intake.setPower(1);
        follower.followPath(intakePath,0.7,false);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }
    public void shoot(ElapsedTime timer, int count){
        C920PanelsEOCV.C920Pipeline.SlotState color = C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;
        if(count == pattern - 21){
            color = C920PanelsEOCV.C920Pipeline.SlotState.GREEN;
        }
        if(shooting == 0) {
            for (int i = 0; i < 3; i++) {
                if (colors[i] == color) {
                    int o = (robot.cpos+i+1) % 3;
                    robot.setCycle(o);
                    break;
                }
            }
            shooting = 1;
        }
        else if(timer.seconds() >= cycleTime && timer.seconds() < cycleTime + outTime){
            robot.transferUp();
        }
        else if(timer.seconds() >= cycleTime + outTime &&  timer.seconds() < cycleTime + outTime + transferTime){
            robot.transferDown();
        }
        while(timer.seconds() < cycleTime + outTime + transferTime){
            follower.update(); // delay
        }
    }
    public void shootThree(){
        ElapsedTime shootTimer = new ElapsedTime();
        follower.followPath(shootGPP);
        while(opModeIsActive() && follower.isBusy()){
            follower.update();
        }
        shoot(shootTimer,0);
        shoot(shootTimer,1);
        shoot(shootTimer,2);
    }
    public void autonomousPathUpdate(){
        if(opModeTimer.seconds() > 29.5){
            pathState = PathState.STOP;
        }
        else{
            robot.outtake('r');
        }
        switch (pathState){
            case PRELOAD:
                follower.followPath(shootPreload, true);
                shootThree();
                pathState = PathState.GPP;
                break;
            case GPP:
                intakeThree(preIntakeGPP,intakeGPP);
                shootThree();
                pathState = PathState.PARK;
                break;
            case PARK:
                follower.followPath(park);
                if(!follower.isBusy()){
                    pathState = PathState.STOP;
                }
            case STOP:
                robot.launch.setPower(0);
                robot.setCycle(0);
                robot.setIntakePower(0);
                break;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);
        pathState = PathState.PRELOAD;
        opModeTimer = new ElapsedTime();
        limelight = robot.getLimelight();
        limelight.start();
        buildPaths();
        while(this.opModeInInit()) {
            opModeTimer.reset();
            // Limelight setup

            pattern = 21;
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                if (id >= 21 && id <= 23) {
                    pattern = id;
                }
            }
        }
        waitForStart();
        while(this.opModeIsActive()){
            colors = robot.pipeline.getSlotStates();
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("Path State", pathState);
            telemetry.addData("Pattern", pattern);
            telemetry.update();
        }
    }
}
