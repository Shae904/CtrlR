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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Red Close 12")
public class RedCloseTwelve extends LinearOpMode {

    // ===== robot + pedro =====
    public static Robot robot;
    public Follower follower;
    public Limelight3A limelight;

    // ===== timing / behavior =====
    public static double INTAKE_FULL = -1;   // same direction as your RedFarSix update()
    public static double INTAKE_HALF = -0.5; // conserve voltage while traveling to shoot

    // shot macro timings (use these instead of Robot.cycleTime/outTime/transferTime)
    public static double CYCLE_SETTLE = Robot.FIRE_CYCLE_SETTLE_TIME; // let cycler servo move a bit
    public static double FEED_TIME    = Robot.FIRE_FEED_TIME; // HOLD transferUp (increase if it "barely swings up")
    public static double DOWN_TIME    = Robot.FIRE_DOWN_TIME; // time between shots (transfer down)

    // ===== pattern =====
    // tag 21..23 -> pattern; green index = (pattern - 21) (0..2)
    public static int pattern = 21;

    // ===== auto aim =====
    public static int apriltagPipeline = 0;
    public static int speakerTagIdRed = 24;

    public static double aimMaxTurn = 0.6;
    public static double aimTimeoutSec = 1.2;

    // ===== aimlock while moving to shoot (override heading once tag is seen) =====
    public static double AIMLOCK_MAX_DRIVE = 0.55;   // translation cap while aimlock is active
    public static double AIMLOCK_KP_FIELD  = 0.045;  // position P in FIELD inches -> drive command
    public static double AIMLOCK_POS_TOL   = 1.6;    // inches to consider "at shoot pose"
    public static double AIMLOCK_TIMEOUT   = 1.4;    // seconds max once aimlock starts (per shoot leg)
    public static double PREAIM_MIN_TIME   = 0.0;    // keep 0 unless you want to force some aimlock time

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

        double out = Robot.AIM_Kp * err
                + Robot.AIM_Ki * aimInteg
                + Robot.AIM_Kd * deriv
                + Robot.AIM_Ks * Math.signum(err);

        return Range.clip(out, -1.0, 1.0);
    }

    /**
     * Short aim "nudge" used right before shooting to let heading catch up.
     * If tag visible, apply PID turn for up to `seconds`; else hold still.
     */
    private void microAimAtSpeakerTag(double seconds) {
        resetAimPid();
        try { limelight.pipelineSwitch(apriltagPipeline); } catch (Exception ignored) {}

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && t.seconds() < seconds) {
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);

            Double tx = getTxForTag(speakerTagIdRed);
            if (tx != null) {
                double turn = aimPidFromTx(tx, Robot.AIM_OFFSET_RED);
                turn = Range.clip(turn, -aimMaxTurn, aimMaxTurn);

                robot.fl.setPower( turn);
                robot.bl.setPower( turn);
                robot.fr.setPower(-turn);
                robot.br.setPower(-turn);
            } else {
                robot.fl.setPower(0);
                robot.bl.setPower(0);
                robot.fr.setPower(0);
                robot.br.setPower(0);
            }

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
                            new Pose(123.974, 119.682),
                            new Pose(102.728,  97.510)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(45))
                    .build();

            FIRSTSHOOTTOINTAKEPPG = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(102.728,  97.510),
                            new Pose(116.649,  87.113),
                            new Pose(129.046,  73.470)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-59), Math.toRadians(0))
                    .build();

            PPGTOSECONDSHOOT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(129.046,  73.470),
                            new Pose( 94.391,  88.980)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            SECONDSHOOTTOPGP = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 94.391,  88.980),
                            new Pose( 85.646,  51.930),
                            new Pose(135.642,  58.311)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            PGPTOTHIRDSHOOT = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(135.642,  58.311),
                            new Pose(116.291,  54.808),
                            new Pose( 86.649,  82.099)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            THIRDSHOOTTOGPP = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 86.649,  82.099),
                            new Pose( 81.603,  19.060),
                            new Pose(134.358,  37.119)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            GPPTOLASTSHOOT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.358,  37.119),
                            new Pose( 95.298,  88.417)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            PARK = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 95.298,  88.417),
                            new Pose( 99.020,  84.636)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();
        }
    }

    private Paths paths;

    // ===== shoot poses (endpoints of the shoot legs) =====
    private static final Pose FIRST_SHOOT_POSE  = new Pose(102.728, 97.510, Math.toRadians(45));
    private static final Pose SECOND_SHOOT_POSE = new Pose( 94.391, 88.980, Math.toRadians(45));
    private static final Pose THIRD_SHOOT_POSE  = new Pose( 86.649, 82.099, Math.toRadians(45));
    private static final Pose LAST_SHOOT_POSE   = new Pose( 95.298, 88.417, Math.toRadians(45));

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

    /** Drive helper (robot-centric x/y + rx) */
    private void setDrivePowers(double rotY, double rotX, double rx) {
        rotX *= 1.1; // keep your teleop behavior

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double fl = (rotY + rotX + rx) / denom;
        double bl = (rotY - rotX + rx) / denom;
        double fr = (rotY - rotX - rx) / denom;
        double br = (rotY + rotX - rx) / denom;

        robot.fl.setPower(fl);
        robot.bl.setPower(bl);
        robot.fr.setPower(fr);
        robot.br.setPower(br);
    }

    /** Aimlock override while still calling follower.update() for localization. */
    private void aimlockDriveToward(Pose targetPose, Double txOrNull) {
        // Update pose estimate via follower's localizer (we call follower.update() in the loop)
        Pose cur = follower.getPose();
        double ex = targetPose.getX() - cur.getX();
        double ey = targetPose.getY() - cur.getY();

        // Field vector scaled
        double vxField = Range.clip(AIMLOCK_KP_FIELD * ex, -AIMLOCK_MAX_DRIVE, AIMLOCK_MAX_DRIVE);
        double vyField = Range.clip(AIMLOCK_KP_FIELD * ey, -AIMLOCK_MAX_DRIVE, AIMLOCK_MAX_DRIVE);

        // Convert field -> robot frame using IMU heading
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = vxField * Math.cos(-botHeading) - vyField * Math.sin(-botHeading);
        double rotY = vxField * Math.sin(-botHeading) + vyField * Math.cos(-botHeading);

        // Rotation from tag (if visible)
        double rx = 0.0;
        if (txOrNull != null) {
            rx = aimPidFromTx(txOrNull, Robot.AIM_OFFSET_RED);
            rx = Range.clip(rx, -aimMaxTurn, aimMaxTurn);
        }

        setDrivePowers(rotY, rotX, rx);
    }

    /** Shoot ONE ball using your pattern + OpenCV slot logic. */
    private void shootOne(int count) {
        if (robot.pipeline == null) return;

        // choose which color we need for this shot
        C920PanelsEOCV.C920Pipeline.SlotState want =
                (count == (pattern - 21))
                        ? C920PanelsEOCV.C920Pipeline.SlotState.GREEN
                        : C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

        // read slots
        C920PanelsEOCV.C920Pipeline.SlotState[] colors = robot.pipeline.getSlotStates();
        if (colors == null || colors.length < 3) return;

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
            robot.updateCycle();
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            sleep(10);
        }

        // transfer up (feed)
        t.reset();
        robot.transferUp();
        while (opModeIsActive() && t.seconds() < FEED_TIME) {
            robot.updateCycle();
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            sleep(10);
        }

        // transfer down + spacing
        robot.transferDown();
        t.reset();
        while (opModeIsActive() && t.seconds() < DOWN_TIME) {
            robot.updateCycle();
            robot.outtake('r');
            robot.intake.setPower(INTAKE_HALF);
            sleep(10);
        }
    }

    private void shootThree() {
        // small settle + heading catch-up (~300ms)
        microAimAtSpeakerTag(0.10);

        shootOne(0);
        shootOne(1);
        shootOne(2);
    }

    /**
     * Follow a shoot path with Pedro heading UNTIL the speaker tag is seen.
     * Once tag is seen, we "override" heading by taking over drivetrain (aimlock) while still
     * calling follower.update() so localization keeps up.
     *
     * Intake always runs (half), flywheel always on, and we preselect the FIRST ball while moving.
     */
    private void followToShoot(PathChain path, Pose shootPose) {
        follower.followPath(path, true);

        boolean aimLock = false;
        ElapsedTime aimLockTimer = new ElapsedTime();
        resetAimPid();

        while (opModeIsActive() && follower.isBusy()) {

            // always update pedro (pose estimate)
            follower.update();
            robot.updateCycle();

            // flywheel always on
            robot.outtake('r');

            // half intake while traveling to shoots
            robot.intake.setPower(INTAKE_HALF);

            // keep pattern fresh
            pattern = readPatternFromLimelight(pattern);

            // while moving to shoot: pre-select the first shot
            preSelectForShot(0);

            // check tag
            Double tx = getTxForTag(speakerTagIdRed);

            if (!aimLock && tx != null) {
                // FIRST time we see the tag: switch into aimlock override
                aimLock = true;
                resetAimPid();
                aimLockTimer.reset();
            }

            if (aimLock) {
                // If we lose the tag later, keep translation control but rotation = 0
                aimlockDriveToward(shootPose, tx);

                // stop condition: close enough OR timeout (so we never stall)
                Pose cur = follower.getPose();
                double dist = Math.hypot(shootPose.getX() - cur.getX(), shootPose.getY() - cur.getY());

                if ((aimLockTimer.seconds() >= PREAIM_MIN_TIME && dist <= AIMLOCK_POS_TOL)
                        || (aimLockTimer.seconds() >= AIMLOCK_TIMEOUT)) {
                    // let pedro finish "busy" by continuing the loop;
                    // but we can force stop driving now:
                    setDrivePowers(0, 0, 0);
                    break;
                }
            }

            telemetry.addData("state", state);
            telemetry.addData("pattern", pattern);
            telemetry.addData("aimLock", aimLock ? "on" : "off");
            telemetry.addData("tx", tx == null ? "null" : String.format("%.2f", tx));
            telemetry.update();
        }

        // stop drivetrain at end of leg
        setDrivePowers(0, 0, 0);

        // If Pedro is still "busy" for a moment, give it a couple updates to settle pose.
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            follower.update();
            robot.updateCycle();
            sleep(10);
        }
    }

    /** Follow an intake path; intake FULL (same direction/speed behavior as your RedFarSix). */
    private void followIntake(PathChain path) {
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            robot.updateCycle();
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
        follower.setStartingPose(new Pose(123.974, 119.682, Math.toRadians(135)));

        limelight = robot.getLimelight();
        limelight.start();
        limelight.pipelineSwitch(apriltagPipeline);

        // ===== INIT: read pattern (21..23) =====
        pattern = 21;
        while (opModeInInit()) {
            pattern = readPatternFromLimelight(pattern);
            robot.setCycle(0);
            robot.updateCycle();
            robot.transferDown();
            robot.intake.setPower(0);

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
                    followToShoot(paths.FROMSTARTTOFIRSTSHOOT, FIRST_SHOOT_POSE);
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
                    followToShoot(paths.PPGTOSECONDSHOOT, SECOND_SHOOT_POSE);
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
                    followToShoot(paths.PGPTOTHIRDSHOOT, THIRD_SHOOT_POSE);
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
                    followToShoot(paths.GPPTOLASTSHOOT, LAST_SHOOT_POSE);
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
                        robot.updateCycle();
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
        setDrivePowers(0, 0, 0);
    }
}
