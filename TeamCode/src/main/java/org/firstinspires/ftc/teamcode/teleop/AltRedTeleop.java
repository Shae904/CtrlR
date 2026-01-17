package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Robot;
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
@TeleOp(name = "Alt Red Teleop")
public class AltRedTeleop extends LinearOpMode {

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

    // ================= aim pid (panels) =================
    public static double kp = 0.016;
    public static double ki = 0.0;
    public static double kd = 0.0017;
    public static double ks = 0.06;
    public static double deadband = 0.4;

    public static double aimOffsetDeg = 0.0; // if ur limelight is not perfectly centered
    public static double maxTurn = 1.0;      // clamp rx
    public static int apriltagPipeline = 0;  // limelight apriltag pipeline index
    public static int targetTagId = 24;      // red speaker tag

    public static boolean aimOnRightBumper = true; // if false, aim is off unless u code another button
    public static boolean fieldCentric = false;    // ctrlr currently robot centric; flip this if u want
    public static double rotStickScale = 1.0;      // scale manual rx

    // local pid state (dont put this in robot.java unless u want it global)
    private double integ = 0.0;
    private double lastErr = 0.0;
    private long lastNanos = 0L;

    private void resetPid() {
        integ = 0.0;
        lastErr = 0.0;
        lastNanos = 0L;
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

        telemetry.addLine("right bumper = aim assist");
        telemetry.update();

        waitForStart();

        int lastPipeline = -1;
        resetPid();

        while (opModeIsActive()) {

            // switch pipeline only when changed
            if (apriltagPipeline != lastPipeline) {
                robot.limelight.pipelineSwitch(apriltagPipeline);
                lastPipeline = apriltagPipeline;
            }

            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            // drive sticks
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            // manual rotate (both drivers if u want)
            double rxManual = (gamepad1.right_stick_x + gamepad2.right_stick_x) * rotStickScale;

            // ===== aim assist overrides rx =====
            boolean aimOn = aimOnRightBumper && gamepad1.right_bumper;

            double rx;
            Double tx = null;

            if (aimOn) {
                tx = getTxForTag(targetTagId);

                if (tx == null) {
                    // no tag, dont freak out
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

            // ===== robot centric (default) or field centric =====
            double rotX = x;
            double rotY = y;

            if (fieldCentric) {
                double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1; // same thing u had commented before
            } else {
                // keep ur old strafe fix
                rotX *= 1.1;
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

            // flywheel always running like u had
            double out = robot.outtake('r');

            // ================== controls / state machine ==================
            if (gamepad2.a) {
                state = RunState.INTAKE;
                shooting = 0;
            } else if (gamepad2.x) {
                state = RunState.SHOOT0;
                if (shooting == 0) shootTime.reset();
            } else if (gamepad2.y) {
                state = RunState.SHOOT1;
                if (shooting == 0) shootTime.reset();
            } else if (gamepad2.b) {
                state = RunState.SHOOT2;
                if (shooting == 0) shootTime.reset();
            }

            if (gamepad2.left_bumper) {
                robot.transferUp();
            } else {
                robot.transferDown();
            }

            switch (state) {
                case INTAKE:
                    robot.setCycle(0);
                    robot.transferDown();

                    if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
                        robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                    } else if (gamepad2.right_trigger > 0.05 || gamepad2.left_trigger > 0.05) {
                        robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    } else {
                        robot.intake.setPower(0);
                    }
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

            // ================== telemetry ==================
            telemetry.addData("aim", aimOn ? "on" : "off");
            telemetry.addData("tx", (tx == null ? "null" : String.format("%.2f", tx)));
            telemetry.addData("rx", String.format("%.2f", rx));
            telemetry.addData("kp", kp);
            telemetry.addData("kd", kd);
            telemetry.addData("ks", ks);
            telemetry.addData("deadband", deadband);

            telemetry.addData("target velocity", out);
            telemetry.addData("current velocity", robot.launch.getVelocity());
            telemetry.update();
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

    // ================= helpers =================

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

        // deadband = dont fight noise at center
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
