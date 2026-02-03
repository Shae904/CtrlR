package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name = "Axon Analog PIDF (CR, Same Power, Linear)", group = "Tuning")
public class CyclePIDFTuner extends LinearOpMode {

    // =========================
    // Dashboard-tunable fields
    // =========================

    // Target selection
    public static int targetIndex = 0;

    // =========================
    // Internal state
    // =========================
    private double integral = 0.0;
    private double lastErr = 0.0;

    private double spPos = 0.0;

    private double spVel = 0.0;

    @Override
    public void runOpMode() {

        // ----- Hardware -----
        CRServo cycle1 = hardwareMap.get(CRServo.class, "cycle1");
        CRServo cycle2 = hardwareMap.get(CRServo.class, "cycle2");
        AnalogInput servoPos;
        try {
            servoPos = hardwareMap.get(AnalogInput.class, "servoAnalog");
        } catch (Exception ignored) {
            servoPos = hardwareMap.get(AnalogInput.class, "servoPos");
        }

        ElapsedTime timer = new ElapsedTime();

        // Initialize setpoint to current position
        double voltage = servoPos.getVoltage();
        double pos = normalizeVoltage(voltage);
        spPos = pos;
        spVel = 0.0;

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        timer.reset();

        boolean lastA = false, lastB = false, lastX = false, lastY = false;

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();
            if (dt <= 0) dt = 0.02;

            // ----- Read position -----
            voltage = servoPos.getVoltage();
            pos = normalizeVoltage(voltage);
            int nearestIdx = getNearestPresetIndex(pos);

            // ----- Gamepad helpers -----
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            if (a && !lastA) targetIndex = (targetIndex + 1) % Math.max(1, SpinSorter.presetPositions.length);
            if (b && !lastB) targetIndex = (targetIndex - 1 + Math.max(1, SpinSorter.presetPositions.length)) % Math.max(1, SpinSorter.presetPositions.length);

            // Capture calibration
            if (x && !lastX) SpinSorter.minV = voltage;
            if (y && !lastY) SpinSorter.maxV = voltage;

            lastA = a; lastB = b; lastX = x; lastY = y;

            // ----- Target -----
            int idx = Range.clip(targetIndex, 0, Math.max(0, SpinSorter.presetPositions.length - 1));
            double target = getPresetPos(idx);

            // ----- Motion profile -----
            if (SpinSorter.enableControl) {
                updateProfile(target, dt);
            } else {
                spPos = pos;
                spVel = 0.0;
                integral = 0.0;
                lastErr = 0.0;
            }

            // ----- Error -----
            double err = positionError(spPos, pos);
            if (Math.abs(err) < SpinSorter.positionDeadband) {
                err = 0.0;
            }

            // ----- PID -----
            integral += err * dt;
            integral = Range.clip(integral, -SpinSorter.iClamp, SpinSorter.iClamp);

            double deriv = (err - lastErr) / dt;
            lastErr = err;

            double uPid = SpinSorter.kP * err + SpinSorter.kI * integral + SpinSorter.kD * deriv;

            // ----- Feedforward -----
            double uFf = 0.0;
            if (Math.abs(spVel) > 1e-6) uFf += Math.signum(spVel) * SpinSorter.kS;
            uFf += SpinSorter.kV * spVel;

            double out = uPid + uFf;
            if (SpinSorter.invertOutput) out = -out;

            // ----- Clamp & deadband -----
            out = Range.clip(out, -SpinSorter.maxPower, SpinSorter.maxPower);
            if (Math.abs(out) < SpinSorter.outputDeadband) out = 0.0;

            // =================================================
            // EXACT SAME POWER TO BOTH SERVOS — ALWAYS
            // =================================================
            cycle1.setPower(out);
            cycle2.setPower(out);

            // ----- Telemetry -----
            telemetry.addData("nearest", "%d / %d", nearestIdx, Math.max(0, SpinSorter.presetPositions.length - 1));
            telemetry.addData("targetIndex", "%d / %d", idx, Math.max(0, SpinSorter.presetPositions.length - 1));
            telemetry.addData("target", "%.3f", target);
            telemetry.addData("pos", "%.3f", pos);
            telemetry.addData("spPos/spVel", "%.3f / %.3f", spPos, spVel);
            telemetry.addData("err", "%.3f", err);
            telemetry.addData("out", "%.3f", out);
            telemetry.addData("PID", "P %.2f  I %.2f  D %.2f", SpinSorter.kP, SpinSorter.kI, SpinSorter.kD);
            telemetry.addData("FF", "kS %.2f  kV %.2f", SpinSorter.kS, SpinSorter.kV);
            telemetry.addData("V", "%.3f", voltage);
            telemetry.addData("minV/maxV", "%.3f / %.3f", SpinSorter.minV, SpinSorter.maxV);
            telemetry.update();
        }
    }

    // =========================
    // Helper functions
    // =========================

    private double normalizeVoltage(double v) {
        double pos = CycleWheelMath.normalizeVoltage(v, SpinSorter.minV, SpinSorter.maxV);
        if (SpinSorter.invertSensor) pos = 1.0 - pos;
        return (pos >= 1.0) ? 0.0 : pos;
    }

    private double positionError(double target, double current) {
        return CycleWheelMath.circleError(target, current);
    }

    private void updateProfile(double target, double dt) {
        double e = positionError(target, spPos);

        double desiredVel = Range.clip(e / Math.max(dt, 0.02), -SpinSorter.maxVel, SpinSorter.maxVel);

        double dvMax = SpinSorter.maxAccel * dt;
        spVel += Range.clip(desiredVel - spVel, -dvMax, dvMax);
        spVel = Range.clip(spVel, -SpinSorter.maxVel, SpinSorter.maxVel);

        spPos += spVel * dt;

        spPos = CycleWheelMath.wrap01(spPos);
    }

    private double getPresetPos(int presetIdx) {
        if (SpinSorter.presetPositions == null || SpinSorter.presetPositions.length == 0) return 0.0;
        int idx = Range.clip(presetIdx, 0, SpinSorter.presetPositions.length - 1);
        return CycleWheelMath.wrap01(SpinSorter.presetPositions[idx]);
    }

    public int getNearestPresetIndex(double posNorm) {
        return CycleWheelMath.nearestIndexOnCircle(SpinSorter.presetPositions, posNorm);
    }

    /**
     * Set the current target index. This only changes {@link #targetIndex}; the
     * motion and wrap-around behavior are still governed by the existing profile + PID.
     */
    public void setTargetIndex(int idx) {
        targetIndex = Range.clip(idx, 0, Math.max(0, SpinSorter.presetPositions.length - 1));
    }
}
