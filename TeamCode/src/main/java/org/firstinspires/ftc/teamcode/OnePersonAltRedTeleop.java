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
@TeleOp(name = "One Person Alt Red Teleop")
public class OnePersonAltRedTeleop extends LinearOpMode {

    public static Robot robot;

    public enum RunState {
        SHOOT0,
        SHOOT1,
        SHOOT2,
        INTAKE
    }

    public RunState state;

    // ===== aim pid (panels) =====
    public static double kp = 0.016;
    public static double ki = 0.0;
    public static double kd = 0.0017;
    public static double ks = 0.06;
    public static double deadband = 0.4;

    public static double aimOffsetDeg = 0.0;
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

    // ===== MACRO TIMING (panels tunable) =====
    // extra wait before opening transfer (your request)
    public static double feedDelayAfterAligned = 0.20;

    // how long transfer stays up for one shot
    public static double feedUpTime = 0.16;

    // time after shot w transfer down before next shot
    public static double betweenShotsDown = 0.35;

    // safety: if analog never agrees, don't softlock forever
    public static double alignTimeout = 0.80;

    // ===== DPAD LEFT: fixed fire test order 2 -> 0 -> 1 =====
    private enum FireTestState {
        IDLE,
        MOVE_2, WAIT_2, FEED_2, DOWN_2,
        MOVE_0, WAIT_0, FEED_0, DOWN_0,
        MOVE_1, WAIT_1, FEED_1, DOWN_1,
        DONE
    }

    private FireTestState ftState = FireTestState.IDLE;
    private final ElapsedTime ftTimer = new ElapsedTime();
    private boolean ftActive = false;

    // analog “stable reads” counter
    private int stableReads = 0;

    // ===== DPAD DOWN: shoot 3 using sorting logic =====
    private enum Sort3State {
        IDLE,
        PICK_SHOT, MOVE, WAIT_ALIGN, FEED, DOWN,
        DONE
    }

    private Sort3State s3State = Sort3State.IDLE;
    private final ElapsedTime s3Timer = new ElapsedTime();
    private boolean s3Active = false;
    private int s3ShotsDone = 0;
    private int s3TargetCycle = -1;

    // edge detect dpads
    private boolean lastDpadLeft = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);

        robot = new Robot(this);
        robot.limelight.start();
        robot.limelight.pipelineSwitch(apriltagPipeline);

        state = RunState.INTAKE;
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("1p alt red:");
        telemetry.addLine("rb = aim assist");
        telemetry.addLine("lb = transfer (manual) unless macro running");
        telemetry.addLine("rt/lt = intake in/out");
        telemetry.addLine("x/y/b = set cycle 0/1/2 (manual)");
        telemetry.addLine("dpad left = fixed fire test 2->0->1 (analog gated)");
        telemetry.addLine("dpad down = shoot 3 using sorting logic (analog gated)");
        telemetry.addLine("dpad right = imu reset");
        telemetry.update();

        waitForStart();

        int lastPipeline = -1;
        resetPid();

        while (opModeIsActive()) {

            // only switch limelight pipeline when changed
            if (apriltagPipeline != lastPipeline) {
                robot.limelight.pipelineSwitch(apriltagPipeline);
                lastPipeline = apriltagPipeline;
            }

            // imu reset
            if (gamepad1.dpad_right) {
                robot.imu.resetYaw();
            }

            // ===== drive sticks =====
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rxManual = gamepad1.right_stick_x * rotStickScale;

            // ===== aim overrides rx when rb held =====
            boolean aimOn = gamepad1.right_bumper;

            double rx;
            Double tx = null;

            if (aimOn) {
                tx = getTxForTag(targetTagId);
                if (tx == null) {
                    resetPid();
                    rx = rxManual;
                } else {
                    rx = pidFromTx(tx, aimOffsetDeg);
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

            // ===== macro triggers (edge detect) =====
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

            // cancel macro
            if ((ftActive || s3Active) && gamepad1.a) {
                stopAllMacros();
            }

            // ===== run active macros =====
            if (ftActive) {
                runFireTestStep();
            } else if (s3Active) {
                runSort3Step();
            }

            // if macro active, it owns cycle + transfer.
            // still allow intake motor to be OFF to avoid fighting
            if (ftActive || s3Active) {
                robot.intake.setPower(0);

                telemetry.addData("macro", ftActive ? ("FIRETEST " + ftState) : ("SORT3 " + s3State));
            } else {
                telemetry.addData("macro", "none");

                // ===== normal manual controls =====

                // manual state buttons (x/y/b just set cycle positions)
                if (gamepad1.x) {
                    state = RunState.SHOOT0;
                } else if (gamepad1.y) {
                    state = RunState.SHOOT1;
                } else if (gamepad1.b) {
                    state = RunState.SHOOT2;
                } else if (gamepad1.a) {
                    state = RunState.INTAKE;
                }

                // transfer manual MUST ALWAYS WORK here
                if (gamepad1.left_bumper) {
                    robot.transferUp();
                } else {
                    robot.transferDown();
                }

                // intake
                double in = gamepad1.right_trigger;
                double outTrig = gamepad1.left_trigger;
                double intakePow = outTrig - in;

                if (Math.abs(intakePow) > 0.05) robot.intake.setPower(intakePow);
                else robot.intake.setPower(0);

                // cycle selection
                switch (state) {
                    case INTAKE:
                        robot.setCycle(0);
                        break;
                    case SHOOT0:
                        robot.setCycle(1);
                        break;
                    case SHOOT1:
                        robot.setCycle(2);
                        break;
                    case SHOOT2:
                        robot.setCycle(0);
                        break;
                }
            }

            // ===== telemetry =====
            telemetry.addData("state", state);
            telemetry.addData("aim", aimOn ? "on" : "off");
            telemetry.addData("tx", (tx == null ? "null" : String.format("%.2f", tx)));
            telemetry.addData("rx", String.format("%.2f", rx));

            telemetry.addData("cycleIdx(cpos)", robot.cpos);
            telemetry.addData("cycleV", robot.hasCycleAnalog() ? String.format("%.2f", robot.getCycleVoltage()) : "no analog");

            telemetry.addData("target vel", target);
            telemetry.addData("current vel", robot.launch.getVelocity());
            telemetry.update();

            sleep(20);
        }

        // cleanup
        try {
            if (robot.webcam != null) {
                robot.webcam.stopStreaming();
                robot.webcam.closeCameraDevice();
            }
        } catch (Exception ignored) { }

        try {
            robot.limelight.close();
        } catch (Exception ignored) { }
    }

    // =======================
    // DPAD LEFT: FIRE TEST 2->0->1 (analog gated)
    // =======================

    private void startFireTest() {
        ftActive = true;
        ftState = FireTestState.MOVE_2;
        ftTimer.reset();
        stableReads = 0;
        robot.transferDown();
    }

    private void stopFireTest() {
        ftActive = false;
        ftState = FireTestState.IDLE;
        robot.transferDown();
    }

    private void stopAllMacros() {
        stopFireTest();
        stopSort3();
    }

    private boolean alignedStable(int cycleIndex) {
        if (!robot.hasCycleAnalog()) return true; // no analog -> don't block
        boolean ok = robot.cycleAtIndex(cycleIndex);
        if (ok) stableReads++;
        else stableReads = 0;
        return stableReads >= Robot.CYCLE_STABLE_COUNT;
    }

    private void runFireTestStep() {
        switch (ftState) {
            case MOVE_2:
                robot.setCycle(2);
                robot.transferDown();
                ftTimer.reset();
                stableReads = 0;
                ftState = FireTestState.WAIT_2;
                break;

            case WAIT_2:
                if (alignedStable(2) || ftTimer.seconds() >= alignTimeout) {
                    ftTimer.reset(); // reuse timer for delay+feed
                    ftState = FireTestState.FEED_2;
                }
                break;

            case FEED_2:
                // extra wait before opening transfer
                if (ftTimer.seconds() < feedDelayAfterAligned) break;
                robot.transferUp();
                if (ftTimer.seconds() >= feedDelayAfterAligned + feedUpTime) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN_2;
                }
                break;

            case DOWN_2:
                if (ftTimer.seconds() >= betweenShotsDown) {
                    ftState = FireTestState.MOVE_0;
                }
                break;

            case MOVE_0:
                robot.setCycle(0);
                robot.transferDown();
                ftTimer.reset();
                stableReads = 0;
                ftState = FireTestState.WAIT_0;
                break;

            case WAIT_0:
                if (alignedStable(0) || ftTimer.seconds() >= alignTimeout) {
                    ftTimer.reset();
                    ftState = FireTestState.FEED_0;
                }
                break;

            case FEED_0:
                if (ftTimer.seconds() < feedDelayAfterAligned) break;
                robot.transferUp();
                if (ftTimer.seconds() >= feedDelayAfterAligned + feedUpTime) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN_0;
                }
                break;

            case DOWN_0:
                if (ftTimer.seconds() >= betweenShotsDown) {
                    ftState = FireTestState.MOVE_1;
                }
                break;

            case MOVE_1:
                robot.setCycle(1);
                robot.transferDown();
                ftTimer.reset();
                stableReads = 0;
                ftState = FireTestState.WAIT_1;
                break;

            case WAIT_1:
                if (alignedStable(1) || ftTimer.seconds() >= alignTimeout) {
                    ftTimer.reset();
                    ftState = FireTestState.FEED_1;
                }
                break;

            case FEED_1:
                if (ftTimer.seconds() < feedDelayAfterAligned) break;
                robot.transferUp();
                if (ftTimer.seconds() >= feedDelayAfterAligned + feedUpTime) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN_1;
                }
                break;

            case DOWN_1:
                if (ftTimer.seconds() >= betweenShotsDown) {
                    ftState = FireTestState.DONE;
                }
                break;

            case DONE:
            default:
                stopFireTest();
                break;
        }
    }

    // =======================
    // DPAD DOWN: SORTING SHOOT 3 (uses RedTeleop-style selection)
    // =======================

    private void startSort3() {
        // needs pipeline for sorting, otherwise it's pointless
        if (robot.pipeline == null) return;

        s3Active = true;
        s3State = Sort3State.PICK_SHOT;
        s3Timer.reset();
        stableReads = 0;

        s3ShotsDone = 0;
        s3TargetCycle = -1;

        robot.transferDown();
    }

    private void stopSort3() {
        s3Active = false;
        s3State = Sort3State.IDLE;
        s3ShotsDone = 0;
        s3TargetCycle = -1;
        robot.transferDown();
    }

    private C920PanelsEOCV.C920Pipeline.SlotState[] getColorsSafe() {
        if (robot.pipeline == null) return null;
        return robot.pipeline.getSlotStates();
    }

    // RedTeleop-style selection:
    // scan i=0..2, if colors[i] == want, setCycle((cpos + i + 1) % 3)
    private int pickCycleForColor(C920PanelsEOCV.C920Pipeline.SlotState want) {
        C920PanelsEOCV.C920Pipeline.SlotState[] colors = getColorsSafe();
        if (colors == null) return -1;

        // if the "fire slot" is already correct, don't rotate
        int fireSlot = robot.getFireSlot();
        if (colors[fireSlot] == want) {
            return robot.cpos; // no change needed
        }

        for (int i = 0; i < 3; i++) {
            if (colors[i] == want) {
                return robot.cycleIndexForSlotIndex(i); // (cpos + i + 1) % 3
            }
        }
        return -1;
    }

    // decide what to shoot THIS shot:
    // if any PURPLE exists, shoot PURPLE, else if any GREEN exists, shoot GREEN, else stop.
    private C920PanelsEOCV.C920Pipeline.SlotState chooseNextColorToShoot() {
        C920PanelsEOCV.C920Pipeline.SlotState[] colors = getColorsSafe();
        if (colors == null) return null;

        boolean hasPurple = false;
        boolean hasGreen = false;

        for (int i = 0; i < 3; i++) {
            if (colors[i] == C920PanelsEOCV.C920Pipeline.SlotState.PURPLE) hasPurple = true;
            if (colors[i] == C920PanelsEOCV.C920Pipeline.SlotState.GREEN) hasGreen = true;
        }

        if (hasPurple) return C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;
        if (hasGreen)  return C920PanelsEOCV.C920Pipeline.SlotState.GREEN;
        return null;
    }

    private void runSort3Step() {
        switch (s3State) {
            case PICK_SHOT: {
                if (s3ShotsDone >= 3) {
                    s3State = Sort3State.DONE;
                    break;
                }

                C920PanelsEOCV.C920Pipeline.SlotState want = chooseNextColorToShoot();
                if (want == null) {
                    s3State = Sort3State.DONE;
                    break;
                }

                int cyclePick = pickCycleForColor(want);
                if (cyclePick < 0) {
                    s3State = Sort3State.DONE;
                    break;
                }

                s3TargetCycle = cyclePick;
                s3State = Sort3State.MOVE;
                break;
            }

            case MOVE:
                robot.transferDown();
                if (s3TargetCycle != robot.cpos) {
                    robot.setCycle(s3TargetCycle);
                }
                s3Timer.reset();
                stableReads = 0;
                s3State = Sort3State.WAIT_ALIGN;
                break;

            case WAIT_ALIGN:
                if (alignedStable(s3TargetCycle) || s3Timer.seconds() >= alignTimeout) {
                    s3Timer.reset();
                    s3State = Sort3State.FEED;
                }
                break;

            case FEED:
                // extra wait before opening transfer
                if (s3Timer.seconds() < feedDelayAfterAligned) break;

                robot.transferUp();
                if (s3Timer.seconds() >= feedDelayAfterAligned + feedUpTime) {
                    robot.transferDown();
                    s3Timer.reset();
                    s3State = Sort3State.DOWN;
                }
                break;

            case DOWN:
                if (s3Timer.seconds() >= betweenShotsDown) {
                    s3ShotsDone++;
                    s3State = Sort3State.PICK_SHOT;
                }
                break;

            case DONE:
            default:
                stopSort3();
                break;
        }
    }

    // =======================
    // Limelight + PID helpers
    // =======================

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

        if (Math.abs(err) < deadband) {
            integ = 0.0;
            lastErr = err;
            return 0.0;
        }

        integ += err * dt;
        double deriv = (err - lastErr) / dt;
        lastErr = err;

        double out = kp * err + ki * integ + kd * deriv + ks * Math.signum(err);
        return Range.clip(out, -1.0, 1.0);
    }
}
