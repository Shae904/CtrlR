package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Blue Far 9")
public class BlueFarNine extends LinearOpMode {
    public static Robot robot;
    public static ElapsedTime opModeTimer;

    public static double cycleTime = Robot.cycleTime;
    public static double outTime = Robot.outTime;
    public static double transferTime = Robot.transferTime;

    public static double[][] PoseCoords = {
            {57,9,90}, // Start
            {54,10,125}, // Shoot
            {61,84,180}, // Intake PPG Start
            {30,84,180}, // Intake PPG End
            {61,60,180}, // Intake PGP Start
            {30,60,180}, // Intake PGP End
            {61,36,180}, // Intake GPP Start
            {30,36,180}, // Intake GPP End
            {50,50,128} // Park
    };
    public static Pose START,SHOOT,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END,PARK;
    public Follower follower;
    public static int pattern = 21;
    public Limelight3A limelight;
    private int shooting;
    public C920PanelsEOCV.C920Pipeline.SlotState[] colors;

    public enum PathState{PRELOAD,PGP,GPP,PARK,STOP}

    private PathState pathState = PathState.PRELOAD;

    public void initializePoses(){
        START = new Pose(PoseCoords[0][0],PoseCoords[0][1],Math.toRadians(PoseCoords[0][2]));
        SHOOT = new Pose(PoseCoords[1][0],PoseCoords[1][1],Math.toRadians(PoseCoords[1][2]));
        INTAKE_PGP_START = new Pose(PoseCoords[4][0],PoseCoords[4][1],Math.toRadians(PoseCoords[4][2]));
        INTAKE_PGP_END = new Pose(PoseCoords[5][0],PoseCoords[5][1],Math.toRadians(PoseCoords[5][2]));
        INTAKE_GPP_START = new Pose(PoseCoords[6][0],PoseCoords[6][1],Math.toRadians(PoseCoords[6][2]));
        INTAKE_GPP_END = new Pose(PoseCoords[7][0],PoseCoords[7][1],Math.toRadians(PoseCoords[7][2]));
        PARK = new Pose(PoseCoords[8][0],PoseCoords[8][1],Math.toRadians(PoseCoords[8][2]));
    }

    public PathChain shootPreload,preIntakePPG,intakePPG,launchPPG,preIntakePGP,intakePGP,shootPGP,preIntakeGPP,intakeGPP,shootGPP,park;
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
                .build();*/
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
        shootPGP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PGP_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_PGP_START.getHeading(),SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();

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
            update();
        }
        robot.intake.setPower(1);
        follower.followPath(intakePath,0.5,false);
        while (opModeIsActive() && follower.isBusy()) {
            update();
        }
    }
    public void shoot(int count){
        C920PanelsEOCV.C920Pipeline.SlotState color = C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;
        if(count == pattern - 21){
            color = C920PanelsEOCV.C920Pipeline.SlotState.GREEN;
        }
        for (int i = 2; i > -1; i--) {
            if (colors[i] == color) {
                int o = (robot.cpos+i+1) % 3;
                robot.setCycle(o);
                break;
            }
        }
        ElapsedTime timer = new ElapsedTime();
        boolean up = false;
        boolean down = false;
        while(opModeIsActive() && timer.seconds() < cycleTime + outTime + transferTime) {
            robot.outtake('b');
            if (!up && timer.seconds() >= cycleTime) {
                robot.transferUp();
                up = true;
            }
            if(!down && timer.seconds() >= cycleTime + outTime){
                robot.transferDown();
                down = true;
            }
            update();
        }
    }
    public void shootThree(){
        while(opModeIsActive() && follower.isBusy()){
            update();
        }
        shoot(0);
        shoot(1);
        shoot(2);
    }
    public void autonomousPathUpdate(){
        if(opModeTimer.seconds() > 29.5){
            pathState = PathState.STOP;
        }
        switch (pathState){
            case PRELOAD:
                follower.followPath(shootPreload, true);
                while(opModeIsActive() && follower.isBusy()){update();}
                shootThree();
                pathState = PathState.GPP;
                break;
            case GPP:
                intakeThree(preIntakeGPP,intakeGPP);
                follower.followPath(shootGPP);
                while(opModeIsActive() && follower.isBusy()){update();}
                shootThree();
                pathState = PathState.PGP;
                break;
            case PGP:
                intakeThree(preIntakePGP,intakePGP);
                follower.followPath(shootPGP);
                while(opModeIsActive() && follower.isBusy()){update();}
                shootThree();
                pathState = PathState.PGP;
                break;
            case PARK:
                follower.followPath(park);
                while(opModeIsActive() && follower.isBusy()){update();}
                pathState = PathState.STOP;
                break;
            case STOP:
                robot.launch.setPower(0);
                robot.setCycle(0);
                robot.setIntakePower(0);
                break;
        }
    }
    public void update(){
        follower.update();
        robot.outtake('r');
        robot.intake.setPower(-0.9);
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id >= 21 && id <= 23) {
                pattern = id;
            }
        }
        telemetry.addData("Path State", pathState);
        telemetry.addData("Pattern", pattern);
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(START);
        pathState = PathState.PRELOAD;
        opModeTimer = new ElapsedTime();
        limelight = robot.getLimelight();
        limelight.start();
        pattern = 21;
        while(this.opModeInInit()) {
            robot.setCycle(0);
            opModeTimer.reset();
            // Limelight setup
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
            autonomousPathUpdate();
        }
    }
}
