package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Closed-loop controller for a 3-chamber "spin sorter" / cycle wheel driven by two CRServos
 * with an analog absolute position sensor (Axon Mini+ analog output, potentiometer, etc.).
 *
 * Position is normalized into a unit circle: [0, 1). (0 and 1 represent the same physical angle.)
 */
@Configurable
public class SpinSorter {

    // =========================
    // Panels tunables (shared)
    // =========================

    // Analog calibration
    public static double minV = 0.10;
    public static double maxV = 3.20;
    public static boolean invertSensor = false;

    // Motor direction (use if your loop runs away / sign is backwards)
    public static boolean invertOutput = false;

    /**
     * Preset "stop" positions on the unit circle [0,1).
     * Keep these sorted (increasing), with the last entry near 1.0 if you use wrap-around.
     */
    public static double[] presetPositions = {
            0.096,
            0.260,
            0.430,
            0.595,
            0.764,
            0.936
    };

    // Enable controller output
    public static boolean enableControl = true;

    // Motion profile limits (normalized units/sec)
    public static double maxVel = 1.5;
    public static double maxAccel = 4.0;

    // PID gains (normalized position -> power)
    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.15;

    // Feedforward (optional)
    public static double kS = 0.0;
    public static double kV = 0.0;

    // Output limits
    public static double maxPower = 1.0;
    public static double outputDeadband = 0.0;
    public static double positionDeadband = 0.01;
    public static double iClamp = 0.25;

    // =========================
    // Hardware
    // =========================

    private final CRServo cycle1;
    private final CRServo cycle2;
    private final AnalogInput analog;

    // =========================
    // Internal controller state
    // =========================

    private final ElapsedTime timer = new ElapsedTime();

    private double targetPos = 0.0;
    private int targetIndex = 0;

    private double spPos = 0.0;
    private double spVel = 0.0;

    private double integral = 0.0;
    private double lastErr = 0.0;

    public SpinSorter(CRServo cycle1, CRServo cycle2, AnalogInput analog) {
        this.cycle1 = cycle1;
        this.cycle2 = cycle2;
        this.analog = analog;
        resetToCurrent();
    }

    public void resetToCurrent() {
        double pos = getPosition01();
        targetPos = pos;
        targetIndex = getNearestPresetIndex();
        spPos = pos;
        spVel = 0.0;
        integral = 0.0;
        lastErr = 0.0;
        timer.reset();
    }

    public double getVoltage() {
        return analog.getVoltage();
    }

    public double getPosition01() {
        double raw = CycleWheelMath.normalizeVoltage(analog.getVoltage(), minV, maxV);
        if (invertSensor) raw = 1.0 - raw;
        // Keep in [0,1) for circle math (treat 1.0 same as 0.0)
        return (raw >= 1.0) ? 0.0 : raw;
    }

    public int getNearestPresetIndex() {
        return CycleWheelMath.nearestIndexOnCircle(presetPositions, getPosition01());
    }

    public static int midPresetIndex() {
        if (presetPositions == null || presetPositions.length == 0) return 0;
        return presetPositions.length / 2;
    }

    public int getTargetIndex() {
        return targetIndex;
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void setTargetPos(double targetPos01) {
        targetPos = CycleWheelMath.wrap01(targetPos01);
    }

    public double getPresetPos(int presetIdx) {
        if (presetPositions == null || presetPositions.length == 0) return 0.0;
        int idx = Range.clip(presetIdx, 0, presetPositions.length - 1);
        return CycleWheelMath.wrap01(presetPositions[idx]);
    }

    public void setTargetPresetIndex(int presetIdx) {
        if (presetPositions == null || presetPositions.length == 0) {
            targetIndex = 0;
            setTargetPos(0.0);
            return;
        }
        int idx = Range.clip(presetIdx, 0, presetPositions.length - 1);
        targetIndex = idx;
        setTargetPos(presetPositions[idx]);
    }

    public static int stepIndex(int fromIndex, int delta, int length) {
        if (length <= 0) return 0;
        int out = (fromIndex + delta) % length;
        if (out < 0) out += length;
        return out;
    }

    public int stepPresetIndex(int fromIndex, int delta) {
        int len = (presetPositions == null) ? 0 : presetPositions.length;
        return stepIndex(fromIndex, delta, len);
    }

    public boolean atTarget() {
        double err = CycleWheelMath.circleAbsError(targetPos, getPosition01());
        return err <= positionDeadband;
    }

    /**
     * Call this every loop to update the motion profile + PID and command both servos.
     */
    public void update() {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.02;

        double pos = getPosition01();

        if (!enableControl) {
            // Hold current position (no output) and clear integrators
            spPos = pos;
            spVel = 0.0;
            integral = 0.0;
            lastErr = 0.0;
            setPower(0.0);
            return;
        }

        // ----- Motion profile on the circle -----
        double eSp = CycleWheelMath.circleError(targetPos, spPos);

        double desiredVel = Range.clip(eSp / Math.max(dt, 0.02), -maxVel, maxVel);

        double dvMax = maxAccel * dt;
        spVel += Range.clip(desiredVel - spVel, -dvMax, dvMax);
        spVel = Range.clip(spVel, -maxVel, maxVel);

        spPos = CycleWheelMath.wrap01(spPos + spVel * dt);

        // ----- Error (circle) -----
        double err = CycleWheelMath.circleError(spPos, pos);
        if (Math.abs(err) < positionDeadband) err = 0.0;

        // ----- PID -----
        integral += err * dt;
        integral = Range.clip(integral, -iClamp, iClamp);

        double deriv = (err - lastErr) / dt;
        lastErr = err;

        double uPid = kP * err + kI * integral + kD * deriv;

        // ----- Feedforward -----
        double uFf = 0.0;
        if (Math.abs(spVel) > 1e-6) uFf += Math.signum(spVel) * kS;
        uFf += kV * spVel;

        double out = uPid + uFf;

        if (invertOutput) out = -out;

        out = Range.clip(out, -maxPower, maxPower);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        setPower(out);
    }

    public void setPower(double power) {
        cycle1.setPower(power);
        cycle2.setPower(power);
    }
}
