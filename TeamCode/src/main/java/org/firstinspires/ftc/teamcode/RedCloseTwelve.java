package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Red Close 12")
public class RedCloseTwelve extends LinearOpMode {

    // ===== robot + pedro =====
    public static Robot robot;
    public Follower follower;
    public Limelight3A limelight;

    // ===== timing / behavior =====
    public static double INTAKE_FULL = -1;  // same direction as your RedFarSix update()
    public static double INTAKE_HALF = -0.5;  // conserve voltage while traveling to shoot

    // shot macro timings (use these instead of Robot.cycleTime/outTime/transferTime)
    public static double CYCLE_SETTLE = 0.12;  // let cycler servo move a bit
    public static double FEED_DELAY   = 0.25;  // wait before transferUp (your teleop macro style)
    public static double FEED_TIME    = 0.28;  // HOLD transferUp (increase if it "barely swings up")
    public static double DOWN_TIME    = 0.40;  // time between shots (transfer down)

    // ===== pattern =====
    // tag 21..23 -> pattern; green index = (pattern - 21)  (0..2)
    public static int pattern = 21;

    // ===== auto aim =====
    public static int apriltagPipeline = 0;
    public static int speakerTagIdRed = 24;

    public static double aimMaxTurn = 0.6;
    public static double aimTimeoutSec = 1.2;

    private double aimInteg = 0.0;
    private double aimLastErr = 0.0;
    private long aimLastNanos = 0L;

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
            if (f.getFiducialId() == id) return f.getTargetXDegrees();
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

    /** Aim in place right before shooting. */
    private void aimAtSpeakerTag() {
        resetAimPid();
        try { limelight.pipelineSwitch(apriltagPipeline); } catch (Exception ignored) {}

        ElapsedTime t = new ElapsedTime();
        int stableCount = 0;
        final int neededStable = 5;

        while (opModeIsActive() && t.seconds() < aimTimeoutSec) {
            // keep flywheel spun
            robot.outtake('r');

            Double tx = getTxForTag(speakerTagIdRed);
            if (tx == null) break;

            double err = tx - Robot.AIM_OFFSET_RED;
            if (Math.abs(err) < Robot.AIM_DEADBAND) stableCount++;
            else stableCount = 0;

            if (stableCount >= neededStable) break;

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

    // ===== paths (your coords) =====
    public static class Paths {
        public PathChain FROMSTARTTOFIRSTSHOOT;
        public PathChain FIRSTSHOOTTOINTAKEPPG;
        public PathChain PPGTOSECONDSHOOT;
        public PathChain SECONDSHOOTTOPGP;
        public PathChain PGPTOTHIRDSHOOT;
        public PathChain THIRDSHOOTTOGPP;
        public PathChain GPPTOLASTSHOOT;
        public PathChain PARK;

        public Paths(Follower follower) {

            FROMSTARTTOFIRSTSHOOT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.927, 120.636),
                            new Pose( 97.722, 105.854)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(45))
                    .build();

            FIRSTSHOOTTOINTAKEPPG = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 97.722, 105.854),
                            new Pose(115.219,  84.967),
                            new Pose(129.523,  73.470)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-59), Math.toRadians(0))
                    .build();

            PPGTOSECONDSHOOT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(129.523,  73.470),
                            new Pose( 86.285,  93.272)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            SECONDSHOOTTOPGP = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 86.285,  93.272),
                            new Pose( 85.646,  51.930),
                            new Pose(135.642,  58.311)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            PGPTOTHIRDSHOOT = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(135.642,  58.311),
                            new Pose(116.291,  54.808),
                            new Pose( 81.881,  88.298)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            THIRDSHOOTTOGPP = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 81.881,  88.298),
                            new Pose( 82.079,  26.212),
                            new Pose(134.358,  37.119)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            GPPTOLASTSHOOT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.358,  37.119),
                            new Pose( 87.430,  95.093)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            PARK = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.430, 95.093),
                            new Pose(86.384, 104.424)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    private Paths paths;

    // ===== state machine =====
    private enum State {
        START_TO_FIRST_SHOOT,
        FIRST_SHOOT,

        FIRST_SHOOT_TO_PPG,
        PPG_TO_SECOND_SHOOT,
        SECOND_SHOOT,

        SECOND_SHOOT_TO_PGP,
        PGP_TO_THIRD_SHOOT,
        THIRD_SHOOT,

        THIRD_SHOOT_TO_GPP,
        GPP_TO_LAST_SHOOT,
        LAST_SHOOT,

        PARK,
        STOP
    }

    private State state = State.START_TO_FIRST_SHOOT;
    private final ElapsedTime opTimer = new ElapsedTime();

    // ===== helpers =====
    private int readPatternFromLimelight(int current) {
        int p = current;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return p;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return p;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (id >= 21 && id <= 23) p = id;
        }
        return p;
    }

    /** Pick the cycle position for the *next* shot (count 0..2) while we are moving. */
    private void preSelectForShot(int count) {
        if (robot.pipeline == null) return;

        C920PanelsEOCV.C920Pipeline.SlotState[] colors = robot.pipeline.getSlotStates();
        if (colors == null || colors.length < 3) return;

        C920PanelsEOCV.C920Pipeline.SlotState want =
                (count == (pattern - 21))
                        ? C920PanelsEOCV.C920Pipeline.SlotState.GREEN
                        : C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

        // same search order as your auton: i=2->0
        int picked = -1;
        for (int i = 2; i >= 0; i--) {
            if (colors[i] == want) { picked = i; break; }
        }
        if (picked == -1) return;

        int targetCycle = (robot.cpos + picked + 1) % 3;
        robot.setCycle(targetCycle);
    }

    /** Shoot ONE ball using your pattern + OpenCV slot logic. */
    private void shootOne(int count) {
        // choose which color we need for this shot
        C920PanelsEOCV.C920Pipeline.SlotState want =
                (count == (pattern - 21))
                        ? C920PanelsEOCV.C920Pipeline.SlotState.GREEN
                        : C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

        // read slots
        C920PanelsEOCV.C920Pipeline.SlotState[] colors = robot.pipeline.getSlotStates();

        // pick i=2->0 like your RedFarSix
        int picked = -1;
        for (int i = 2; i >= 0; i--) {
            if (colors[i] == want) { picked = i; break; }
        }

        // if we can't find it, just fall back to "whatever" (don’t deadlock)
        int targetCycle;
        if (picked == -1) targetCycle = robot.cpos;
        else targetCycle = (robot.cpos + picked + 1) % 3;

        robot.setCycle(targetCycle);

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < CYCLE_SETTLE) {
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            sleep(10);
        }

        // wait before feeding
        t.reset();
        while (opModeIsActive() && t.seconds() < FEED_DELAY) {
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            robot.transferDown();
            sleep(10);
        }

        // transfer up (feed)
        t.reset();
        robot.transferUp();
        while (opModeIsActive() && t.seconds() < FEED_TIME) {
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            sleep(10);
        }

        // transfer down + spacing
        robot.transferDown();
        t.reset();
        while (opModeIsActive() && t.seconds() < DOWN_TIME) {
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            sleep(10);
        }
    }

    private void shootThree() {
        // aim right before the volley
        aimAtSpeakerTag();
        shootOne(0);
        shootOne(1);
        shootOne(2);
    }

    /** Follow a path; keep intake running; preselect first shot (count=0) while moving to shoot. */
    private void followToShoot(PathChain path) {
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // flywheel always on
            robot.outtake('r');

            // half speed intake while traveling to shoots
            robot.intake.setPower(INTAKE_HALF);

            // keep the pattern fresh (optional, but harmless)
            pattern = readPatternFromLimelight(pattern);

            // while moving to shoot: pre-select the first shot
            preSelectForShot(0);

            telemetry.addData("state", state);
            telemetry.addData("pattern", pattern);
            telemetry.update();
        }
    }

    /** Follow a path; intake FULL (same “direction/speed” behavior as your RedFarSix). */
    private void followIntake(PathChain path) {
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            robot.outtake('r');

            // FULL intake on intake legs
            robot.intake.setPower(INTAKE_FULL);

            pattern = readPatternFromLimelight(pattern);

            telemetry.addData("state", state);
            telemetry.addData("pattern", pattern);
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        follower = Constants.createFollower(hardwareMap);

        paths = new Paths(follower);

        // starting pose must match the first point in FROMSTARTTOFIRSTSHOOT
        follower.setStartingPose(new Pose(124.927, 120.636, Math.toRadians(139)));

        limelight = robot.getLimelight();
        limelight.start();
        limelight.pipelineSwitch(apriltagPipeline);

        // ===== INIT: lock pattern (21..23) =====
        pattern = 21;
        while (opModeInInit()) {
            pattern = readPatternFromLimelight(pattern);
            robot.setCycle(0);
            robot.transferDown();
            robot.intake.setPower(INTAKE_HALF);

            telemetry.addData("pattern(tag 21-23)", pattern);
            telemetry.addData("meaning", pattern == 21 ? "GPP" : (pattern == 22 ? "PGP" : "PPG"));
            telemetry.update();
        }

        waitForStart();
        opTimer.reset();

        state = State.START_TO_FIRST_SHOOT;

        while (opModeIsActive() && state != State.STOP) {
            // safety end
            if (opTimer.seconds() > 29.5) state = State.STOP;

            switch (state) {

                case START_TO_FIRST_SHOOT:
                    followToShoot(paths.FROMSTARTTOFIRSTSHOOT);
                    state = State.FIRST_SHOOT;
                    break;

                case FIRST_SHOOT:
                    shootThree();
                    state = State.FIRST_SHOOT_TO_PPG;
                    break;

                case FIRST_SHOOT_TO_PPG:
                    followIntake(paths.FIRSTSHOOTTOINTAKEPPG);
                    state = State.PPG_TO_SECOND_SHOOT;
                    break;

                case PPG_TO_SECOND_SHOOT:
                    followToShoot(paths.PPGTOSECONDSHOOT);
                    state = State.SECOND_SHOOT;
                    break;

                case SECOND_SHOOT:
                    shootThree();
                    state = State.SECOND_SHOOT_TO_PGP;
                    break;

                case SECOND_SHOOT_TO_PGP:
                    followIntake(paths.SECONDSHOOTTOPGP);
                    state = State.PGP_TO_THIRD_SHOOT;
                    break;

                case PGP_TO_THIRD_SHOOT:
                    followToShoot(paths.PGPTOTHIRDSHOOT);
                    state = State.THIRD_SHOOT;
                    break;

                case THIRD_SHOOT:
                    shootThree();
                    state = State.THIRD_SHOOT_TO_GPP;
                    break;

                case THIRD_SHOOT_TO_GPP:
                    followIntake(paths.THIRDSHOOTTOGPP);
                    state = State.GPP_TO_LAST_SHOOT;
                    break;

                case GPP_TO_LAST_SHOOT:
                    followToShoot(paths.GPPTOLASTSHOOT);
                    state = State.LAST_SHOOT;
                    break;

                case LAST_SHOOT:
                    shootThree();
                    state = State.PARK;
                    break;

                case PARK:
                    follower.followPath(paths.PARK, true);
                    while (opModeIsActive() && follower.isBusy()) {
                        follower.update();
                        robot.outtake('r');
                        robot.intake.setPower(INTAKE_HALF);
                        telemetry.addData("state", state);
                        telemetry.update();
                    }
                    state = State.STOP;
                    break;

                default:
                    state = State.STOP;
                    break;
            }
        }

        // stop outputs
        robot.launch.setPower(0);
        robot.intake.setPower(0);
        robot.transferDown();
        robot.setCycle(0);
    }
}