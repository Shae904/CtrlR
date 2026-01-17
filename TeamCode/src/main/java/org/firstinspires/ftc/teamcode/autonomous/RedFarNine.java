package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleop.C920PanelsEOCV;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Red Far 9")
public class RedFarNine extends LinearOpMode {
    public static Robot robot;
    public static ElapsedTime opModeTimer;

    public static double cycleTime = Robot.cycleTime;
    public static double outTime = Robot.outTime;
    public static double transferTime = Robot.transferTime;

    public static double[][] PoseCoords = {
            {87,9,90}, // Start
            {90,14,71.5}, // Shoot
            {83,84,0}, // Intake PPG Start
            {114,84,0}, // Intake PPG End
            {83,60,0}, // Intake PGP Start
            {114,60,0}, // Intake PGP End
            {96,33,0}, // Intake GPP Start
            {133,33,0}, // Intake GPP End
            {90,50,62} // PARK
    };
    public static Pose START,SHOOT,INTAKE_PPG_START,INTAKE_PPG_END,INTAKE_PGP_START,INTAKE_PGP_END,INTAKE_GPP_START,INTAKE_GPP_END,PARK;
    public Follower follower;
    public static int pattern = 21;
    public Limelight3A limelight;
    private int shooting;
    public C920PanelsEOCV.C920Pipeline.SlotState[] colors;

    // ===== auto aim (red tag 24) =====
    public static int speakerTagIdRed = 24;
    public static double aimMaxTurn = 0.6;
    public static double aimTimeoutSec = 1.2;

    private double aimInteg = 0.0;
    private double aimLastErr = 0.0;
    private long aimLastNanos = 0L;

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
            robot.outtake('r');
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

    private void resetAimPid() {
        aimInteg = 0.0;
        aimLastErr = 0.0;
        aimLastNanos = 0L;
    }

    private Double getTxForTag(int id) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == id) {
                return f.getTargetXDegrees();
            }
        }
        return null;
    }

    private double aimPidFromTx(double txDeg, double offsetDeg) {
        long now = System.nanoTime();
        double dt = (aimLastNanos == 0L) ? 0.02 : (now - aimLastNanos) / 1e9;
        aimLastNanos = now;
        if (dt < 0.001) dt = 0.02;

        double err = txDeg - offsetDeg;

        if (Math.abs(err) < Robot.AIM_DEADBAND) {
            aimInteg = 0.0;
            aimLastErr = err;
            return 0.0;
        }

        aimInteg += err * dt;
        double deriv = (err - aimLastErr) / dt;
        aimLastErr = err;

        double out = Robot.AIM_Kp * err + Robot.AIM_Ki * aimInteg + Robot.AIM_Kd * deriv + Robot.AIM_Ks * Math.signum(err);
        return Range.clip(out, -1.0, 1.0);
    }

    /**
     * Rotate in place until tx ~ 0 (within deadband) or timeout.
     * If tag not visible, it will just bail immediately (so auton doesn't stall).
     */
    private void aimAtSpeakerTag() {
        resetAimPid();

        ElapsedTime t = new ElapsedTime();
        int stableCount = 0;
        final int neededStable = 5;

        while (opModeIsActive() && t.seconds() < aimTimeoutSec) {
            robot.outtake('r');

            Double tx = getTxForTag(speakerTagIdRed);
            if (tx == null) {
                break;
            }

            double err = tx - Robot.AIM_OFFSET_RED;
            if (Math.abs(err) < Robot.AIM_DEADBAND) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            if (stableCount >= neededStable) {
                break;
            }

            double turn = aimPidFromTx(tx, Robot.AIM_OFFSET_RED);
            turn = Range.clip(turn, -aimMaxTurn, aimMaxTurn);

            robot.fl.setPower( turn);
            robot.bl.setPower( turn);
            robot.fr.setPower(-turn);
            robot.br.setPower(-turn);

            sleep(20);
        }

        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }
    public void autonomousPathUpdate(){
        if(opModeTimer.seconds() > 29.5){
            pathState = PathState.STOP;
        }
        switch (pathState){
            case PRELOAD:
                follower.followPath(shootPreload, true);
                while(opModeIsActive() && follower.isBusy()){update();}
                aimAtSpeakerTag();
                shootThree();
                pathState = PathState.GPP;
                break;
            case GPP:
                intakeThree(preIntakeGPP,intakeGPP);
                follower.followPath(shootGPP);
                while(opModeIsActive() && follower.isBusy()){update();}
                aimAtSpeakerTag();
                shootThree();
                pathState = PathState.PGP;
                break;
            case PGP:
                intakeThree(preIntakePGP,intakePGP);
                follower.followPath(shootPGP);
                while(opModeIsActive() && follower.isBusy()){update();}
                aimAtSpeakerTag();
                shootThree();
                pathState = PathState.PARK;
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
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    if (id >= 21 && id <= 23) {
                        pattern = id;
                    }
                }
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
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId(); // The ID number of the fiducial
                        if (id >= 21 && id <= 23) {
                            pattern = id;
                        }
                    }
                }
            }
        }
        waitForStart();
        while(this.opModeIsActive()){
            if (robot.pipeline != null) {
                colors = robot.pipeline.getSlotStates();
            } else {
                colors = new C920PanelsEOCV.C920Pipeline.SlotState[] {
                        C920PanelsEOCV.C920Pipeline.SlotState.EMPTY,
                        C920PanelsEOCV.C920Pipeline.SlotState.EMPTY,
                        C920PanelsEOCV.C920Pipeline.SlotState.EMPTY
                };
            }
            autonomousPathUpdate();
        }
    }
}
