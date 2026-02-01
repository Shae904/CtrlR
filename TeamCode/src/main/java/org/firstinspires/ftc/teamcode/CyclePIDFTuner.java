package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// If this doesn't compile, replace @Configurable with @Config and use:
// import com.acmerobotics.dashboard.config.Config;
@Configurable
@TeleOp(name = "Axon Analog PIDF Tuner", group = "Tuning")
public class CyclePIDFTuner extends OpMode {

    private CRServo servo1, servo2;
    private AnalogInput servoAnalog;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastT = 0.0;

    // -------------------- Tuning Stuff -----------------------------

    // Analog calibration (voltage range corresponding to travel range)
    public static double minV = 0.10;
    public static double maxV = 3.20;

    // If analog wraps (near max jumps to near min), set true
    public static boolean isWraparound = false;

    // Discrete positions to test (normalized 0..1)
    public static double[] positions = new double[] { 0.10, 0.40, 0.70, 0.95 };
    public static int targetIndex = 0;

    // Enable/disable controller output
    public static boolean enableControl = true;

    // Motion profile limits (normalized units/sec)
    public static double maxVel = 1.5;
    public static double maxAccel = 4.0;

    // PID gains
    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.15;

    // Feedforward
    public static double kS = 0.00; // static friction compensation
    public static double kV = 0.00; // velocity feedforward

    // Output shaping
    public static double maxPower = 1.0;
    public static double outputDeadband = 0.0;

    // Integral clamp
    public static double iClamp = 0.25;

    // =========================================================
    // Internal state
    // =========================================================
    private double integral = 0.0;
    private double lastErr = 0.0;

    // Profile setpoint state
    private double spPos = 0.0;
    private double spVel = 0.0;

    // Gamepad edge detection
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;

    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servoAnalog = hardwareMap.get(AnalogInput.class, "servoAnalog");

        timer.reset();
        lastT = timer.seconds();

        // Start setpoint at current measured position to avoid jumps on init
        double pos = readNormalizedPosition();
        spPos = pos;
        spVel = 0.0;
        integral = 0.0;
        lastErr = 0.0;
    }

    @Override
    public void loop() {
        double t = timer.seconds();
        double dt = t - lastT;
        lastT = t;
        if (dt <= 0) dt = 0.02;

        // ----- Read measured position -----
        double voltage = servoAnalog.getVoltage();
        double pos = readNormalizedPositionFromVoltage(voltage);

        // ----- Simple controls for testing -----
        // A: next target, B: previous target
        // X: set minV to current voltage (ONLY when physically at min)
        // Y: set maxV to current voltage (ONLY when physically at max)
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;

        if (a && !lastA) targetIndex = (targetIndex + 1) % positions.length;
        if (b && !lastB) targetIndex = (targetIndex - 1 + positions.length) % positions.length;

        if (x && !lastX) minV = voltage;
        if (y && !lastY) maxV = voltage;

        lastA = a; lastB = b; lastX = x; lastY = y;

        // ----- Target -----
        int idx = Math.max(0, Math.min(targetIndex, positions.length - 1));
        double target = Range.clip(positions[idx], 0.0, 1.0);

        // ----- Motion profile (spPos, spVel) -----
        if (enableControl) {
            updateProfile(target, dt);
        } else {
            spPos = pos;
            spVel = 0.0;
            integral = 0.0;
            lastErr = 0.0;
        }

        // ----- Error (wrap-safe) -----
        double err = positionError(spPos, pos);

        // ----- PID -----
        integral += err * dt;
        integral = Range.clip(integral, -iClamp, iClamp);

        double deriv = (err - lastErr) / dt;
        lastErr = err;

        double uPid = (kP * err) + (kI * integral) + (kD * deriv);

        // ----- Feedforward from profile velocity -----
        double uFf = 0.0;
        if (Math.abs(spVel) > 1e-6) uFf += Math.signum(spVel) * kS;
        uFf += kV * spVel;

        double out = uPid + uFf;

        // ----- Clamp & deadband -----
        out = Range.clip(out, -maxPower, maxPower);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // =========================================================
        // CRITICAL REQUIREMENT: EXACT SAME POWER TO BOTH SERVOS
        // =========================================================
        servo1.setPower(out);
        servo2.setPower(out);

        // ----- Telemetry -----
        telemetry.addData("enableControl", enableControl);
        telemetry.addData("targetIndex", "%d / %d", idx, positions.length - 1);
        telemetry.addData("target", "%.3f", target);
        telemetry.addData("spPos/spVel", "%.3f / %.3f", spPos, spVel);

        telemetry.addData("voltage", "%.3f V", voltage);
        telemetry.addData("pos", "%.3f", pos);
        telemetry.addData("err", "%.3f", err);

        telemetry.addData("out", "%.3f", out);
        telemetry.addData("kP/kI/kD", "%.3f / %.3f / %.3f", kP, kI, kD);
        telemetry.addData("kS/kV", "%.3f / %.3f", kS, kV);

        telemetry.addData("minV/maxV", "%.3f / %.3f", minV, maxV);
        telemetry.addLine("A next | B prev | X set minV | Y set maxV (only at physical min/max!)");
        telemetry.update();
    }

    // =========================
    // Helpers
    // =========================

    private double readNormalizedPosition() {
        return readNormalizedPositionFromVoltage(servoAnalog.getVoltage());
    }

    private double readNormalizedPositionFromVoltage(double v) {
        double denom = (maxV - minV);
        if (Math.abs(denom) < 1e-6) return 0.0;
        double norm = (v - minV) / denom;
        return Range.clip(norm, 0.0, 1.0);
    }

    private double positionError(double target, double current) {
        double e = target - current;
        if (!isWraparound) return e;

        // shortest wrap on [0..1)
        double e2 = e - 1.0;
        double e3 = e + 1.0;

        double a1 = Math.abs(e);
        double a2 = Math.abs(e2);
        double a3 = Math.abs(e3);

        if (a2 < a1 && a2 < a3) return e2;
        if (a3 < a1 && a3 < a2) return e3;
        return e;
    }

    private void updateProfile(double target, double dt) {
        // error between desired target and current setpoint position
        double e = positionError(target, spPos);

        // Desired velocity: try to "use up" distance over ~1 loop, capped.
        // This naturally slows near target.
        double desiredVel = Range.clip(e / Math.max(dt, 0.02), -maxVel, maxVel);

        // Accel-limit: move spVel toward desiredVel
        double dvMax = maxAccel * dt;
        double dv = Range.clip(desiredVel - spVel, -dvMax, dvMax);
        spVel += dv;

        // Velocity cap
        spVel = Range.clip(spVel, -maxVel, maxVel);

        // Integrate to position
        spPos += spVel * dt;

        if (!isWraparound) {
            spPos = Range.clip(spPos, 0.0, 1.0);
        } else {
            spPos = spPos % 1.0;
            if (spPos < 0) spPos += 1.0;
        }
    }
}
