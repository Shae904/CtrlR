package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Red Far", group = "Red")
public class RedFar extends LinearOpMode {
    public static Robot robot;


    public static double[][] PoseCoords = {
            {96,18,90}, // Start
            {72,24,Math.atan(72.0/120.0) * 180.0 / Math.PI}, // Shoot
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
    public static int target = 24;
    public final Telemetry telemetry;
    public static ElapsedTime shootTime;

    public RedFar(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initializePoses(){
        for(int i = 0; i < Poses.length; i++) {
           Poses[i] = new Pose(PoseCoords[i][0],PoseCoords[i][1],PoseCoords[i][2]);
        }
    }

    public PathChain shootPreload,preIntakePPG,intakePPG,launchPPG,preIntakePGP,intakePGP,launchPGP,preIntakeGPP,intakeGPP,launchGPP;
    public void buildPaths(){
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(START,SHOOT))
                .setLinearHeadingInterpolation(START.getHeading(), SHOOT.getHeading())
                .build();
        preIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,INTAKE_PPG_START))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), INTAKE_PPG_START.getHeading())
                .build();
        intakePPG = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PPG_START,INTAKE_PPG_END))
                .setConstantHeadingInterpolation(INTAKE_PPG_START.getHeading())
                .build();
        launchPPG = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PPG_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_PPG_START.getHeading(),SHOOT.getHeading())
                .build();
        preIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,INTAKE_PGP_START))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), INTAKE_PGP_START.getHeading())
                .build();
        intakePGP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PGP_START,INTAKE_PGP_END))
                .setConstantHeadingInterpolation(INTAKE_PGP_START.getHeading())
                .build();
        launchPGP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_PGP_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_PGP_START.getHeading(),SHOOT.getHeading())
                .build();
        preIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT,INTAKE_GPP_START))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), INTAKE_GPP_START.getHeading())
                .build();
        intakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_GPP_START,INTAKE_GPP_END))
                .setConstantHeadingInterpolation(INTAKE_GPP_START.getHeading())
                .build();
        launchGPP = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_GPP_END,SHOOT))
                .setLinearHeadingInterpolation(INTAKE_GPP_START.getHeading(),SHOOT.getHeading())
                .build();
    }

    public void runOpMode() throws InterruptedException {
        //INIT
        buildPaths();
        robot = new Robot(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);
        waitForStart();
        limelight = robot.getLimelight();
        limelight.start();
        // START
        initializePoses();
        // LOOP
        if(opModeIsActive()) {
        }
    }
}
