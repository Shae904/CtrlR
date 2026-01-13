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

    public static double cycleTime = 0.4; // todo tune
    public static double outTime = 0.8;   // todo tune

    private final ElapsedTime shootTime = new ElapsedTime();
    private int shooting = 0;

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

    // ===== fire test macro (dpad left) =====
    // generous timings so it doesn't jam
    public static double ft_cycle_settle = 0.4;   // wait after indexer move
    public static double ft_feed_delay   = 0.25;   // wait BEFORE opening transfer (your request)
    public static double ft_feed_time    = 0.15;   // how long transfer stays up
    public static double ft_down_time    = 0.45;   // time between shots w transfer down

    private enum FireTestState {
        IDLE,
        TO_2, WAIT_2, FEED_2, DOWN_2,
        TO_0, WAIT_0, FEED_0, DOWN_0,
        TO_1, WAIT_1, FEED_1, DOWN_1,
        DONE
    }

    private FireTestState ftState = FireTestState.IDLE;
    private final ElapsedTime ftTimer = new ElapsedTime();
    private boolean ftActive = false;
    private boolean lastDpadLeft = false;

    private void startFireTest() {
        ftActive = true;
        ftState = FireTestState.TO_2;
        ftTimer.reset();
    }

    private void stopFireTest() {
        ftActive = false;
        ftState = FireTestState.IDLE;
        robot.transferDown();
    }

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);

        robot = new Robot(this);
        robot.limelight.start();
        robot.limelight.pipelineSwitch(apriltagPipeline);

        state = RunState.INTAKE;
        shootTime.reset();
        telemetry.setMsTransmissionInterval(50);

        /*
          controls (gamepad1 only):
          - move: left stick
          - rotate: right stick x
          - aim assist: hold right bumper
          - intake: rt = in, lt = out
          - transfer: left bumper = up, else down (unless fire test is running)
          - cycle/select shot: x/y/b = shoot0/1/2, a = back to intake
          - imu reset: dpad right (i moved reset off dpad left since it's fire test now)
          - fire test: dpad left = shoots 2 -> 1 -> 0
        */
        telemetry.addLine("1p teleop: rb aim, rt/lt intake, x/y/b cycle, lb transfer, dpad left fire test");
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

            // imu reset moved so it doesn't conflict w fire test
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

            // ===== fire test trigger (dpad left edge) =====
            boolean dpadLeft = gamepad1.dpad_left;
            if (dpadLeft && !lastDpadLeft && !ftActive) {
                startFireTest();
            }
            lastDpadLeft = dpadLeft;

            // if macro running, it owns transfer/cycle. don't run the normal state machine.
            if (ftActive) {
                // optional: let A cancel macro
                if (gamepad1.a) {
                    stopFireTest();
                } else {
                    runFireTestStep();
                }

                // telemetry still updates
                telemetry.addData("state", state);
                telemetry.addData("firetest", ftState);
                telemetry.addData("aim", aimOn ? "on" : "off");
                telemetry.addData("tx", (tx == null ? "null" : String.format("%.2f", tx)));
                telemetry.addData("rx", String.format("%.2f", rx));
                telemetry.addData("kp", kp);
                telemetry.addData("kd", kd);
                telemetry.addData("ks", ks);
                telemetry.addData("db", deadband);
                telemetry.addData("target vel", target);
                telemetry.addData("current vel", robot.launch.getVelocity());
                telemetry.update();

                sleep(20);
                continue;
            }

            // ===== state buttons (single driver) =====
            if (gamepad1.a) {
                state = RunState.INTAKE;
                shooting = 0;
            } else if (gamepad1.x) {
                state = RunState.SHOOT0;
                if (shooting == 0) shootTime.reset();
            } else if (gamepad1.y) {
                state = RunState.SHOOT1;
                if (shooting == 0) shootTime.reset();
            } else if (gamepad1.b) {
                state = RunState.SHOOT2;
                if (shooting == 0) shootTime.reset();
            }

            // transfer manual (this is your fix: always evaluated every loop)
            if (gamepad1.left_bumper) {
                robot.transferUp();
            } else {
                robot.transferDown();
            }

            // intake power (single driver)
            double in = gamepad1.right_trigger;
            double outTrig = gamepad1.left_trigger;
            double intakePow = outTrig - in;

            switch (state) {
                case INTAKE:
                    robot.setCycle(0);
                    // don't force transferDown here anymore, lb should always work
                    if (Math.abs(intakePow) > 0.05) {
                        robot.intake.setPower(intakePow);
                    } else {
                        robot.intake.setPower(0);
                    }
                    break;

                case SHOOT0:
                    robot.setCycle(1);
                    robot.intake.setPower(0);
                    break;

                case SHOOT1:
                    robot.setCycle(2);
                    robot.intake.setPower(0);
                    break;

                case SHOOT2:
                    robot.setCycle(0);
                    robot.intake.setPower(0);
                    break;
            }

            // telemetry
            telemetry.addData("state", state);
            telemetry.addData("firetest", "idle");
            telemetry.addData("aim", aimOn ? "on" : "off");
            telemetry.addData("tx", (tx == null ? "null" : String.format("%.2f", tx)));
            telemetry.addData("rx", String.format("%.2f", rx));
            telemetry.addData("kp", kp);
            telemetry.addData("kd", kd);
            telemetry.addData("ks", ks);
            telemetry.addData("db", deadband);

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

    // ===== fire test runner =====
    private void runFireTestStep() {
        switch (ftState) {
            case TO_2:
                robot.setCycle(2);
                robot.transferDown();
                ftTimer.reset();
                ftState = FireTestState.WAIT_2;
                break;

            case WAIT_2:
                if (ftTimer.seconds() >= ft_cycle_settle + ft_feed_delay) {
                    ftTimer.reset();
                    ftState = FireTestState.FEED_2;
                }
                break;

            case FEED_2:
                robot.transferUp();
                if (ftTimer.seconds() >= ft_feed_time) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN_2;
                }
                break;

            case DOWN_2:
                if (ftTimer.seconds() >= ft_down_time) {
                    ftState = FireTestState.TO_0;
                }
                break;

            case TO_0:
                robot.setCycle(0);
                robot.transferDown();
                ftTimer.reset();
                ftState = FireTestState.WAIT_0;
                break;

            case WAIT_0:
                if (ftTimer.seconds() >= ft_cycle_settle + ft_feed_delay) {
                    ftTimer.reset();
                    ftState = FireTestState.FEED_0;
                }
                break;

            case FEED_0:
                robot.transferUp();
                if (ftTimer.seconds() >= ft_feed_time) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN_0;
                }
                break;

            case DOWN_0:
                if (ftTimer.seconds() >= ft_down_time) {
                    ftState = FireTestState.TO_1;
                }
                break;

            case TO_1:
                robot.setCycle(1);
                robot.transferDown();
                ftTimer.reset();
                ftState = FireTestState.WAIT_1;
                break;

            case WAIT_1:
                if (ftTimer.seconds() >= ft_cycle_settle + ft_feed_delay) {
                    ftTimer.reset();
                    ftState = FireTestState.FEED_1;
                }
                break;

            case FEED_1:
                robot.transferUp();
                if (ftTimer.seconds() >= ft_feed_time) {
                    robot.transferDown();
                    ftTimer.reset();
                    ftState = FireTestState.DOWN_1;
                }
                break;

            case DOWN_1:
                if (ftTimer.seconds() >= ft_down_time) {
                    ftState = FireTestState.DONE;
                }
                break;

            case DONE:
                stopFireTest();
                break;

            default:
                stopFireTest();
                break;
        }
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
