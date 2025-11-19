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

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Autonomous", group = "Red")
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
    public static Pose START,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END;
    public Pose[] Poses = {START,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END};

    public void initializePoses(){
        for(int i = 0; i < Poses.length; i++) {
           Poses[i] = new Pose(PoseCoords[i][0],PoseCoords[i][1],PoseCoords[i][2]);
        }
    }
    public static int pattern = 21;
    public Limelight3A limelight;
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
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
