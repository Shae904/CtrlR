package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Configurable
@TeleOp(name = "Aim PID Tuner (Panels)", group = "Vision")
public class AimPidTunerTeleop extends LinearOpMode {

    // panels tunables
    public static double kp = 0.016;
    public static double ki = 0.0;
    public static double kd = 0.0017;
    public static double ks = 0.06;
    public static double deadband = 0.4;

    public static double aimOffsetDeg = 0.0;
    public static double maxTurn = 1.0;

    public static int apriltagPipeline = 0;
    public static int targetTagId = 24;

    public static boolean fieldCentric = true;
    public static double rotStickScale = 1.0;

    private Robot robot;
    private Limelight3A limelight;
    private TelemetryManager telemetryM;

    // pid state for tuner
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(this);
        limelight = robot.getLimelight();
        limelight.start();
        limelight.pipelineSwitch(apriltagPipeline);

        telemetry.addLine("right bumper = aim pid (tune in panels)");
        telemetry.update();

        waitForStart();
        resetPid();

        int lastPipeline = -1;

        while (opModeIsActive()) {
            // keep robot constants in sync (optional, but nice for copying into real teleop)
            Robot.AIM_Kp = kp;
            Robot.AIM_Ki = ki;
            Robot.AIM_Kd = kd;
            Robot.AIM_Ks = ks;
            Robot.AIM_DEADBAND = deadband;
            Robot.AIM_OFFSET_RED = aimOffsetDeg;

            // only switch pipelines when value changes
            if (apriltagPipeline != lastPipeline) {
                limelight.pipelineSwitch(apriltagPipeline);
                lastPipeline = apriltagPipeline;
            }

            // drive inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rxManual = (gamepad1.right_stick_x + gamepad2.right_stick_x) * rotStickScale;

            boolean aimOn = gamepad1.right_bumper;

            double rx;
            if (aimOn) {
                Double tx = getTxForTag(targetTagId);
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

            // field centric like your red teleop
            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x;
            double rotY = y;

            if (fieldCentric) {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
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

            // panels debug
            Double txDbg = getTxForTag(targetTagId);
            telemetryM.debug("aim: " + (aimOn ? "on" : "off"));
            telemetryM.debug("pipeline: " + apriltagPipeline + " tag: " + targetTagId + " tx: " +
                    (txDbg == null ? "null" : String.format("%.2f", txDbg)));
            telemetryM.debug("kp " + kp + " ki " + ki + " kd " + kd + " ks " + ks + " db " + deadband);
            telemetryM.debug("offset " + aimOffsetDeg + " maxTurn " + maxTurn);
            telemetryM.debug("rxManual " + String.format("%.2f", rxManual) + " rxOut " + String.format("%.2f", rx));
            telemetryM.update(telemetry);

            sleep(20);
        }
    }

    // grab tx (deg) for one fiducial
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

    private double pidFromTx(double txDeg, double offsetDeg) {
        long now = System.nanoTime();
        double dt = (lastNanos == 0L) ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt < 0.001) dt = 0.02;

        double err = txDeg - offsetDeg;

        if (Math.abs(err) < deadband) {
            // stop + clear i near target so it doesn't wind up
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
