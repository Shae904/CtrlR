package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Configurable
@TeleOp(name = "One Person Red Teleop")
public class OnePersonRedTeleop extends LinearOpMode {

    public static Robot robot;

    // ===== aim pid (shared in Robot.java) =====
    public static double maxTurn = 1.0;

    public static int apriltagPipeline = 0;
    public static int targetTagId = 24;

    public static boolean fieldCentric = false;
    public static double rotStickScale = 1.0;

    // local pid state
    private double integ = 0.0;
    private double lastErr = 0.0;
    private long lastNanos = 0L;

    private void resetPid() {
        integ = 0.0;
        lastErr = 0.0;
        lastNanos = 0L;
    }

    // ===== PATTERN =====
    private int pattern = 21;         // default if no tag seen
    private int lockedPattern = 21;   // used during Sort3 macro
    private boolean patternLocked = false;

    private int readPatternFromLimelight() {
        int p = pattern;

        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) return p;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return p;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (id >= 21 && id <= 23) {
                p = id;
            }
        }
        return p;
    }

    // ===== Fire test macro (DPAD LEFT) =====
    private enum FireTestState {
        IDLE,
        MOVE, WAIT_AT_TARGET, FEED, DOWN,
        NEXT, DONE
    }

    private FireTestState ftState = FireTestState.IDLE;
    private final ElapsedTime ftTimer = new ElapsedTime();
    private boolean ftActive = false;

    private int ftIndex = 0;

    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;

    private void startFireTest() {
        ftActive = true;
        ftState = FireTestState.MOVE;
        ftIndex = 0;
        ftTimer.reset();
    }

    private void stopFireTest() {
        ftActive = false;
        ftState = FireTestState.IDLE;
        robot.transferDown();
    }

    // ===== Sort3 macro (DPAD DOWN) =====
    // Step mapping from camera slot index -> how many preset positions to move.
    // Convention used here:
    //   step > 0 => CCW steps
    //   step < 0 => CW steps
    public static int STEP_FOR_CAM_SLOT0 = -1;
    public static int STEP_FOR_CAM_SLOT1 = 1;
    public static int STEP_FOR_CAM_SLOT2 = 0;

    public static double SORT_MOVE_TIMEOUT = 1.25;

    private enum Sort3State {
        IDLE,
        PICK_AND_MOVE, WAIT_AT_TARGET, FEED, DOWN,
        NEXT, DONE
    }

    private Sort3State s3State = Sort3State.IDLE;
    private final ElapsedTime s3Timer = new ElapsedTime();
    private boolean s3Active = false;

    private int s3Shot = 0;

    private int s3CycleIndex = 0;
    private C920PanelsEOCV.C920Pipeline.SlotState[] s3SavedSlots = null;

    // debug
    private int s3Picked = -1;
    private int s3LastStep = 0;

    private String s3FailReason = "";
    private void startSort3() {
        lockedPattern = readPatternFromLimelight();
        patternLocked = true;
        s3FailReason = "";
        if (robot.pipeline == null) {
            s3FailReason = "pipeline null (vision not init or camera failed)";
            patternLocked = false;
            return;
        }
        C920PanelsEOCV.C920Pipeline.SlotState[] colors = robot.pipeline.getSlotStates();
        if (colors == null || colors.length < 3) {
            s3FailReason = "slotStates null/short";
            patternLocked = false;
            return;
        }

        s3SavedSlots = colors.clone();
        s3CycleIndex = robot.cpos;

        s3Active = true;
        s3State = Sort3State.PICK_AND_MOVE;
        s3Shot = 0;
        s3Timer.reset();

        s3Picked = -1;
        s3LastStep = 0;
    }

    private void stopSort3() {
        s3Active = false;
        s3State = Sort3State.IDLE;
        robot.transferDown();
        patternLocked = false;
        s3SavedSlots = null;
        s3Picked = -1;
        s3LastStep = 0;
        s3FailReason = "";
    }

    // ===== Sort3 helpers =====
    private int stepForPicked(int picked) {
        if (picked == 0) return STEP_FOR_CAM_SLOT0;
        if (picked == 1) return STEP_FOR_CAM_SLOT1;
        if (picked == 2) return STEP_FOR_CAM_SLOT2;
        return 0;
    }

    private void applyCycleStep(int step) {
        int n = Math.abs(step);
        for (int k = 0; k < n; k++) {
            if (step > 0) robot.cycleCCW();
            else if (step < 0) robot.cycleCW();
        }
    }

    private void rotateSavedSlotsByStep(int step) {
        if (s3SavedSlots == null || s3SavedSlots.length < 3) return;
        if (step == 0) return;

        int n = Math.abs(step);
        for (int k = 0; k < n; k++) {
            if (step > 0) {
                // rotate RIGHT: [a,b,c] -> [c,a,b]
                C920PanelsEOCV.C920Pipeline.SlotState c = s3SavedSlots[2];
                s3SavedSlots[2] = s3SavedSlots[1];
                s3SavedSlots[1] = s3SavedSlots[0];
                s3SavedSlots[0] = c;
            } else {
                // rotate LEFT: [a,b,c] -> [b,c,a]
                C920PanelsEOCV.C920Pipeline.SlotState a = s3SavedSlots[0];
                s3SavedSlots[0] = s3SavedSlots[1];
                s3SavedSlots[1] = s3SavedSlots[2];
                s3SavedSlots[2] = a;
            }
        }
    }

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);

        robot = new Robot(this);
        robot.limelight.start();
        robot.limelight.pipelineSwitch(apriltagPipeline);
        telemetry.setMsTransmissionInterval(50);

        final int initCycleIndex = SpinSorter.midPresetIndex();

        while (opModeInInit()) {
            pattern = readPatternFromLimelight();
            robot.setCycle(initCycleIndex);
            robot.updateCycle();
            telemetry.addData("pattern(tag 21-23)", pattern);
            telemetry.addData("meaning", pattern == 21 ? "GPP" : (pattern == 22 ? "PGP" : "PPG"));
            telemetry.addData("cycle idx", "%d / %d", robot.cpos, Math.max(0, SpinSorter.presetPositions.length - 1));
            telemetry.addData("cycle target", "%d", robot.spinSorter.getTargetIndex());
            telemetry.update();
        }

        waitForStart();

        int lastPipeline = -1;
        resetPid();

        while (opModeIsActive()) {

            if (apriltagPipeline != lastPipeline) {
                robot.limelight.pipelineSwitch(apriltagPipeline);
                lastPipeline = apriltagPipeline;
            }

            if (!patternLocked) {
                pattern = readPatternFromLimelight();
            }

            // imu reset (moved off dpad so it doesn't conflict with Sort3)
            if (gamepad1.yWasPressed()) {
                robot.imu.resetYaw();
            }

            // ===== drive sticks =====
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rxManual = gamepad1.right_stick_x * rotStickScale;

            // ===== aim assist (rb) =====
            boolean aimOn = gamepad1.right_bumper;

            Double tx = null;
            double rx;
            if (aimOn) {
                tx = getTxForTag(targetTagId);
                if (tx == null) {
                    resetPid();
                    rx = rxManual;
                } else {
                    rx = pidFromTx(tx, Robot.AIM_OFFSET_RED);
                }
            } else {
                resetPid();
                rx = rxManual;
            }

            rx = Range.clip(rx, -maxTurn, maxTurn);

            double rotX = x;
            double rotY = y;

            if (fieldCentric) {
                double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1;
            } else {
                rotX *= 1.1;
            }

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double fl = (rotY + rotX + rx) / denom;
            double bl = (rotY - rotX + rx) / denom;
            double fr = (rotY - rotX - rx) / denom;
            double br = (rotY + rotX - rx) / denom;

            robot.fl.setPower(fl);
            robot.bl.setPower(bl);
            robot.fr.setPower(fr);
            robot.br.setPower(br);

            // flywheel always on
            double target = robot.outtake('r');

            // ===== macro triggers (edge) =====
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadRight = gamepad1.dpad_right;

            // Fire3 / FireTest on DPAD DOWN
            if (dpadDown && !lastDpadDown && !ftActive && !s3Active) {
                startFireTest();
            }

            // Sort3 on DPAD RIGHT
            if (dpadRight && !lastDpadRight && !ftActive && !s3Active) {
                startSort3();
            }

            lastDpadDown = dpadDown;
            lastDpadRight = dpadRight;

            // allow A to cancel any macro
            if ((ftActive || s3Active) && gamepad1.a) {
                stopFireTest();
                stopSort3();
            }

            // Always keep sorter control loop running (even during macros)
            robot.updateCycle();

            // ===== macro runner ownership =====
            if (ftActive) {
                runFireTestStep();
                telemetryMacro(target, tx, rx, "firetest", ftState.toString(), patternLocked ? lockedPattern : pattern);
                continue;
            }

            if (s3Active) {
                runSort3Step();
                telemetryMacro(target, tx, rx, "sort3", s3State.toString(), lockedPattern);
                continue;
            }

            // ===== normal one-person controls =====

            if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
                robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad1.xWasPressed()) {
                robot.cycleCW();
            } else if (gamepad1.bWasPressed()) {
                robot.cycleCCW();
            }

            // transfer manual (LB)
            if (gamepad1.left_bumper) {
                robot.transferUp();
            } else {
                robot.transferDown();
            }

            // telemetry
            telemetry.addData("pattern(tag 21-23)", pattern);
            telemetry.addData("meaning", pattern == 21 ? "GPP" : (pattern == 22 ? "PGP" : "PPG"));
            telemetry.addData("aim", aimOn ? "on" : "off");
            telemetry.addData("tx", (tx == null ? "null" : String.format("%.2f", tx)));
            telemetry.addData("rx", String.format("%.2f", rx));
            telemetry.addData("kp", Robot.AIM_Kp);
            telemetry.addData("kd", Robot.AIM_Kd);
            telemetry.addData("ks", Robot.AIM_Ks);
            telemetry.addData("Fire3 State", ftState);
            telemetry.addData("Sort3 State", s3State);
            telemetry.addData("Sort3 slots", (s3SavedSlots == null) ? "null" :
                    String.format("%s %s %s", s3SavedSlots[0], s3SavedSlots[1], s3SavedSlots[2]));
            telemetry.addData("Sort3 picked", s3Picked);
            telemetry.addData("Sort3 step", s3LastStep);
            telemetry.addData("", s3FailReason);
            telemetry.addData("db", Robot.AIM_DEADBAND);
            telemetry.addData("target vel", target);
            telemetry.addData("current vel", robot.launch.getVelocity());
            telemetry.update();
        }

        // cleanup
        try { robot.transferDown(); } catch (Exception ignored) { }
        try { robot.intake.setPower(0); } catch (Exception ignored) { }
        try { robot.launch.setPower(0); } catch (Exception ignored) { }

        new Thread(() -> {
            try {
                if (robot.webcam != null) {
                    robot.webcam.stopStreaming();
                    robot.webcam.closeCameraDevice();
                }
            } catch (Exception ignored) { }

            try {
                if (robot.limelight != null) {
                    robot.limelight.close();
                }
            } catch (Exception ignored) { }
        }).start();
    }

    // ===== FireTest =====
    private void runFireTestStep() {
        switch (ftState) {
            case MOVE:
                robot.cycleCW();
                robot.transferDown();
                ftTimer.reset();
                ftState = FireTestState.WAIT_AT_TARGET;
                break;

            case WAIT_AT_TARGET:
                if (robot.spinSorter.atTarget() || ftTimer.seconds() > Robot.FIRE_CYCLE_SETTLE_TIME) {
                    ftTimer.reset();
                    ftState = FireTestState.FEED;
                }
                break;

            case FEED:
                robot.transferUp();
                if (ftTimer.seconds() >= Robot.FIRE_FEED_TIME) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN;
                }
                break;

            case DOWN:
                if (ftTimer.seconds() >= Robot.FIRE_DOWN_TIME) {
                    ftState = FireTestState.NEXT;
                }
                break;

            case NEXT:
                ftIndex++;
                if (ftIndex >= 3) {
                    ftState = FireTestState.DONE;
                } else {
                    ftState = FireTestState.MOVE;
                }
                break;

            case DONE:
            default:
                stopFireTest();
                break;
        }
    }

    // ===== Sort3 =====
    private void runSort3Step() {
        switch (s3State) {
            case PICK_AND_MOVE: {
                if (s3SavedSlots == null || s3SavedSlots.length < 3) {
                    stopSort3();
                    return;
                }

                C920PanelsEOCV.C920Pipeline.SlotState want =
                        (s3Shot == (lockedPattern - 21))
                                ? C920PanelsEOCV.C920Pipeline.SlotState.GREEN
                                : C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

                // prefer shooter (2) then 1 then 0
                int picked = -1;
                for (int i = 2; i >= 0; i--) {
                    if (s3SavedSlots[i] == want) {
                        picked = i;
                        break;
                    }
                }

                s3Picked = picked;

                if (picked < 0) {
                    stopSort3();
                    return;
                }

                int step = stepForPicked(picked);
                s3LastStep = step;

                robot.transferDown();

                // Move cycle
                applyCycleStep(step);

                // Keep internal model consistent
                rotateSavedSlotsByStep(step);

                s3Timer.reset();
                s3State = Sort3State.WAIT_AT_TARGET;
                break;
            }

            case WAIT_AT_TARGET: {
                if (robot.spinSorter.atTarget() || s3Timer.seconds() > SORT_MOVE_TIMEOUT) {
                    s3Timer.reset();
                    s3State = Sort3State.FEED;
                }
                break;
            }

            case FEED: {
                robot.transferUp();
                if (s3Timer.seconds() >= Robot.FIRE_FEED_TIME) {
                    robot.transferDown();
                    if (s3SavedSlots != null && s3SavedSlots.length >= 3) {
                        s3SavedSlots[2] = C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
                    }
                    s3Timer.reset();
                    s3State = Sort3State.DOWN;
                }
                break;
            }

            case DOWN: {
                if (s3Timer.seconds() >= Robot.FIRE_DOWN_TIME) {
                    s3State = Sort3State.NEXT;
                }
                break;
            }

            case NEXT: {
                s3Shot++;
                if (s3Shot >= 3) {
                    s3State = Sort3State.DONE;
                } else {
                    s3State = Sort3State.PICK_AND_MOVE;
                }
                break;
            }

            case DONE:
            default:
                stopSort3();
                break;
        }
    }

    private void telemetryMacro(double target, Double tx, double rx, String macroName, String macroState, int patt) {
        telemetry.addData("macro", macroName);
        telemetry.addData("macroState", macroState);
        telemetry.addData("pattern(tag 21-23)", patt);
        telemetry.addData("meaning", patt == 21 ? "GPP" : (patt == 22 ? "PGP" : "PPG"));
        telemetry.addData("aim", gamepad1.right_bumper ? "on" : "off");
        telemetry.addData("tx", (tx == null ? "null" : String.format("%.2f", tx)));
        telemetry.addData("rx", String.format("%.2f", rx));
        telemetry.addData("target vel", target);
        telemetry.addData("current vel", robot.launch.getVelocity());
        telemetry.addData("Sort3 slots", (s3SavedSlots == null) ? "null" :
                String.format("%s %s %s", s3SavedSlots[0], s3SavedSlots[1], s3SavedSlots[2]));
        telemetry.addData("Sort3 picked", s3Picked);
        telemetry.addData("Sort3 step", s3LastStep);
        telemetry.addData("Sort3 fail", s3FailReason);
        telemetry.update();
    }

    private Double getTxForTag(int id) {
        LLResult result = robot.limelight.getLatestResult();
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

    private double pidFromTx(double txDeg, double offsetDeg) {
        long now = System.nanoTime();
        double dt = (lastNanos == 0L) ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt < 0.001) dt = 0.02;

        double err = txDeg - offsetDeg;

        if (Math.abs(err) < Robot.AIM_DEADBAND) {
            integ = 0.0;
            lastErr = err;
            return 0.0;
        }

        integ += err * dt;
        double deriv = (err - lastErr) / dt;
        lastErr = err;

        double out = Robot.AIM_Kp * err + Robot.AIM_Ki * integ + Robot.AIM_Kd * deriv + Robot.AIM_Ks * Math.signum(err);
        return Range.clip(out, -1.0, 1.0);
    }
}