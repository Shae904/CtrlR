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

@Autonomous(name = "Red Far", group = "Red")
public class RedFar extends LinearOpMode {
    public static Robot robot;


    public static double[][] PoseCoords = {
            {96,9,90}, // Start
            {96,10,Math.atan(48.0/134.0) * 180.0 / Math.PI}, // Shoot
            {83,84,0}, // Intake PPG Start
            {114,84,0}, // Intake PPG End
            {83,60,0}, // Intake PGP Start
            {114,60,0}, // Intake PGP End
            {83,36,0}, // Intake GPP Start
            {114,36,0} // Intake GPP End
    };
    public static Pose START,SHOOT,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END;
    public Pose[] Poses = {START,SHOOT,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END};
    public Follower follower;
    public static int pattern = 21;
    public Limelight3A limelight;
    public final Telemetry telemetry;
    public int[] colors;

    public enum PathState{PRELOAD,PPG,PGP,GPP,STOP}

    private PathState pathState = PathState.PRELOAD;

    public RedFar(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initializePoses(){
        for(int i = 0; i < Poses.length; i++) {
           Poses[i] = new Pose(PoseCoords[i][0],PoseCoords[i][1],Math.toRadians(PoseCoords[i][2]));
        }
    }

    public PathChain shootPreload,preIntakePPG,intakePPG,launchPPG,preIntakePGP,intakePGP,launchPGP,preIntakeGPP,intakeGPP,launchGPP;
    public void buildPaths(){
        initializePoses();
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(START,SHOOT))
                .setLinearHeadingInterpolation(START.getHeading(), SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();

        preIntakePPG = follower.pathBuilder()
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
        launchGPP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_GPP_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_GPP_START.getHeading(),SHOOT.getHeading())
                .setTimeoutConstraint(500)
                .build();
    }

    public void intakeThree(int target){
        // 0 for GPP
        // 1 for PGP
        // 2 for PPG
        if(target == 0){
            follower.followPath(preIntakeGPP);
            if(!follower.isBusy()){

            }
        }
    }

    public void shoot(int target){

    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case PRELOAD:

                if(pattern == 21){
                    robot.setCycle(1);
                }
                else if(pattern == 23){
                    robot.setCycle(2);
                }
                pathState = PathState.STOP;

            case STOP:
                robot.stopOuttake(0);
                robot.setCycle(0);
                robot.setIntakePower(0);
        }
    }

    public void runOpMode() throws InterruptedException {
        //INIT
        buildPaths();

        while(this.opModeInInit()) {
            robot = new Robot(this);
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START);
            waitForStart();

            // Limelight setup
            limelight = robot.getLimelight();
            limelight.start();
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
        while(this.opModeIsActive()){
            autonomousPathUpdate();

            telemetry.addData("Path State", pathState);
            telemetry.addData("Pattern", pattern);
            telemetry.update();
        }
    }
}
