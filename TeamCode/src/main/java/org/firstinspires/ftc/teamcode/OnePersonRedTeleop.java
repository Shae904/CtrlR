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
    // Uses Robot.AIM_Kp/AIM_Ki/AIM_Kd/AIM_Ks/AIM_DEADBAND and Robot.AIM_OFFSET_RED
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

    // ===== PATTERN (same logic as auton) =====
    // tag id 21..23 -> pattern
    // shot count == (pattern - 21) is GREEN, else PURPLE
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
                p = id; // EXACTLY like your auton
            }
        }
        return p;
    }


    // ===== Fire test macro (DPAD LEFT): fixed order 2 -> 0 -> 1 =====
    private enum FireTestState {
        IDLE,
        MOVE, WAIT_AT_TARGET, FEED, DOWN,
        NEXT, DONE
    }

    private FireTestState ftState = FireTestState.IDLE;
    private final ElapsedTime ftTimer = new ElapsedTime();
    private boolean ftActive = false;

    private int ftIndex = 0;

    private boolean lastDpadLeft = false;
    private boolean lastDpadDown = false;

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

    // ===== Sort3 macro (DPAD DOWN): uses OpenCV slot colors + pattern logic =====
    // Step mapping from camera slot index -> how many preset positions to move.
    // Tune these if "left/right" feels swapped.
    public static int STEP_FOR_CAM_SLOT0 = -1;
    public static int STEP_FOR_CAM_SLOT1 = 1;
    public static int STEP_FOR_CAM_SLOT2 = 0;

    // Safety so we never wait forever if the analog is miscalibrated / controller disabled
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

    private boolean cycleCW = false;
    private boolean cycleCCW = false;
    private int s3TargetCycle = 0;
    private int s3CycleIndex = 0;
    private C920PanelsEOCV.C920Pipeline.SlotState[] s3SavedSlots = null;

    private void startSort3() {
        // lock pattern ONCE like auton
        lockedPattern = readPatternFromLimelight();
        patternLocked = true;

        // Snapshot the camera states ONCE for the whole macro (your request)
        if (robot.pipeline == null) {
            stopSort3();
            return;
        }
        C920PanelsEOCV.C920Pipeline.SlotState[] colors = robot.pipeline.getSlotStates();
        if (colors == null || colors.length < 3) {
            stopSort3();
            return;
        }
        s3SavedSlots = colors.clone();
        s3CycleIndex = robot.cpos;

        s3Active = true;
        s3State = Sort3State.PICK_AND_MOVE;
        s3Shot = 0;
        s3Timer.reset();
    }

    private void stopSort3() {
        s3Active = false;
        s3State = Sort3State.IDLE;
        robot.transferDown();
        patternLocked = false;
        s3SavedSlots = null;
    }

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);

        robot = new Robot(this);
        robot.limelight.start();
        robot.limelight.pipelineSwitch(apriltagPipeline);
        telemetry.setMsTransmissionInterval(50);

        final int initCycleIndex = SpinSorter.midPresetIndex();

        // init loop: show pattern like auton does
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

            // pipeline switch only if changed
            if (apriltagPipeline != lastPipeline) {
                robot.limelight.pipelineSwitch(apriltagPipeline);
                lastPipeline = apriltagPipeline;
            }

            // keep updating pattern when NOT locked in a macro
            if (!patternLocked) {
                pattern = readPatternFromLimelight();
            }

            // imu reset
            if (gamepad1.dpad_right) {
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

            // robot centric default, optional field centric
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
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadLeft && !lastDpadLeft && !ftActive && !s3Active) {
                startFireTest();
            }
            if (dpadDown && !lastDpadDown && !ftActive && !s3Active) {
                startSort3();
            }

            lastDpadLeft = dpadLeft;
            lastDpadDown = dpadDown;

            // allow A to cancel any macro
            if ((ftActive || s3Active) && gamepad1.a) {
                stopFireTest();
                stopSort3();
            }

            // Always keep the spin sorter control loop running (even during macros)
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
                robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }else{
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

            // intake (rt in, lt out)
            double in = gamepad1.right_trigger;
            double out = gamepad1.left_trigger;
            double intakePow = out - in;

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
            telemetry.addData("db", Robot.AIM_DEADBAND);
            telemetry.addData("target vel", target);
            telemetry.addData("current vel", robot.launch.getVelocity());
            telemetry.update();
        }

        // cleanup (async so we don't get "stuck in stop()")
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

    // ===== FireTest: 2 -> 0 -> 1 using time-based settle =====
    private void runFireTestStep() {
        switch (ftState) {
            case MOVE:
                robot.cycleCW();
                robot.transferDown();
                ftTimer.reset();
                ftState = FireTestState.WAIT_AT_TARGET;
                break;


            case WAIT_AT_TARGET:
                // Prefer real "at target" lock; fall back to time-based settle
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

    // ===== Sort3: uses your auton/teleop sorting logic + pattern =====
    private void runSort3Step() {
        switch (s3State) {
            case PICK_AND_MOVE: {
                // EXACTLY like auton:
                // green only when count == (pattern - 21)
                C920PanelsEOCV.C920Pipeline.SlotState want =
                        (s3Shot == (lockedPattern - 21))
                                ? C920PanelsEOCV.C920Pipeline.SlotState.GREEN
                                : C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

                // Pick i=2->0. Slot 2 is treated as "current shooter", and slots 0/1 map
                // to "move left" / "move right" by preset index.
                int picked = -1;
                for (int i = 2; i >= 0; i--) {
                    if (s3SavedSlots[i] == want) {
                        picked = i;
                        break;
                    }
                }

                // if we didn't find the wanted color, stop the macro
                if (picked == -1) {
                    stopSort3();
                    return;
                }
                int step = 0;
                if (picked == 0) {
                    robot.cycleCW();
                    step = -1;
                }
                else if (picked == 1) {
                    robot.cycleCCW();
                    step = 1;
                }


                robot.transferDown();

                // Update our saved slot model for the movement so it stays consistent.
                // Right step: slot1 -> slot2, slot0 -> slot1, slot2 -> slot0.
                // Left step:  slot0 -> slot2, slot2 -> slot1, slot1 -> slot0.
                if (s3SavedSlots.length >= 3 && step != 0) {
                    if (step > 0) {
                        C920PanelsEOCV.C920Pipeline.SlotState tmp = s3SavedSlots[2];
                        s3SavedSlots[2] = s3SavedSlots[1];
                        s3SavedSlots[1] = s3SavedSlots[0];
                        s3SavedSlots[0] = tmp;
                    } else if(step <= -1) {
                        C920PanelsEOCV.C920Pipeline.SlotState tmp = s3SavedSlots[0];
                        s3SavedSlots[0] = s3SavedSlots[1];
                        s3SavedSlots[1] = s3SavedSlots[2];
                        s3SavedSlots[2] = tmp;
                    }

                }

                s3Timer.reset();
                s3State = Sort3State.WAIT_AT_TARGET;
                break;
            }

            case WAIT_AT_TARGET: {
                // Prefer real "at target" lock; fall back to time-based settle
                if (robot.spinSorter.atTarget() || s3Timer.seconds() > Robot.FIRE_CYCLE_SETTLE_TIME) {
                    s3Timer.reset();
                    s3State = Sort3State.FEED;
                }
                break;
            }

            case FEED: {
                robot.transferUp();
                if (s3Timer.seconds() >= Robot.FIRE_FEED_TIME) {
                    robot.transferDown();
                    // Mark the fired ball as gone in our saved model
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
