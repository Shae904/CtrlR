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
    public static double minV = 0.008;
    public static double maxV = 3.294;
    public static boolean invertSensor = false;

    /**
     * Preset "stop" positions on the unit circle [0,1).
     * Keep these sorted (increasing), with the last entry near 1.0 if you use wrap-around.
     */
    public static double[] presetPositions = {
            0.100,
            0.258,
            0.428,
            0.593,
            0.762,
            0.936
    };

    // Enable controller output
    public static boolean enableControl = true;

    // Motion profile limits (normalized units/sec)
    public static double maxVel = 1.5;
    public static double maxAccel = 4.0;

    // PID gains (normalized position -> power)
    public static double kP = 0.007;
    public static double kI = 0.0;
    public static double kD = 0.001;

    // Feedforward (optional)
    public static double kF = 0.0;
    public static double MAX_POW = 0.25;
    public static double positionDeadband = 0.01;

    // =========================
    // Hardware
    // =========================

    private final CRServo cycle1;
    private final CRServo cycle2;
    private final AnalogInput analog;

    // =========================
    // Internal controller state
    // =========================
    private boolean approaching = false;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetPos = 0.0;
    private int targetIndex = 0;
    private int lockedDir = 1;

    private double integral = 0.0;
    private double lastErr = 0.0;

    public double pos = 0;
    public double error = 0;

    private boolean engageinBand = false;

    public SpinSorter(CRServo cycle1, CRServo cycle2, AnalogInput analog) {
        this.cycle1 = cycle1;
        this.cycle2 = cycle2;
        this.analog = analog;
        resetToCurrent();
    }

    public void resetToCurrent() {
        targetPos = getPosition();
        targetIndex = midPresetIndex();
        lockedDir = 1;
        integral = 0.0;
        lastErr = 0.0;
        timer.reset();
    }

    public double normalize(double voltage){
        double denom = maxV - minV;
        if (Math.abs(denom) < 1e-9) return 0.0;
        return Range.clip((voltage - minV) / denom, 0.0, 1.0);
    }
    public double getPosition() {
        double p = normalize(analog.getVoltage());
        if (invertSensor) p = 1.0 - p;
        // Keep position on a unit circle [0,1) (treat 1.0 as 0.0)
        if (p >= 1.0) p -= 1.0;
        if (p < 0.0) p += 1.0;
        return p;
    }

    public static int midPresetIndex() {
        if (presetPositions == null || presetPositions.length == 0) return 0;
        return presetPositions.length / 2;
    }

    public int getTargetIndex() {
        return targetIndex;
    }

    public void setTargetPos(double target) {
        if(target != targetPos){
            targetPos = target;
            timer.reset();
            lastErr = 0;
            integral = 0;
        }
    }
    public void cycleCW(){
        int len = Math.max(1, SpinSorter.presetPositions.length);
        int nextIdx = (targetIndex + 1) % len;
        lockedDir = preferredSignFromIndexDelta(targetIndex, nextIdx, len);
        targetIndex = nextIdx;
        setTargetPos(presetPositions[targetIndex]);
    }
    public void cycleCCW(){
        int len = Math.max(1, SpinSorter.presetPositions.length);
        int nextIdx = (targetIndex - 1 + len) % len;
        lockedDir = preferredSignFromIndexDelta(targetIndex, nextIdx, len);
        targetIndex = nextIdx;
        setTargetPos(presetPositions[targetIndex]);
    }
    public void setIndex(int index) {
        if (presetPositions == null || presetPositions.length == 0) return;
        int len = presetPositions.length;
        int idx = index % len;
        if (idx < 0) idx += len;
        lockedDir = preferredSignFromIndexDelta(targetIndex, idx, len);
        targetIndex = idx; //
        setTargetPos(presetPositions[idx]);
    }

    public int stepPresetIndex(int fromIndex, int step) {
        if (presetPositions == null || presetPositions.length == 0) return 0;
        int len = presetPositions.length;
        int start = fromIndex % len;
        if (start < 0) start += len;
        int next = (start + step) % len;
        if (next < 0) next += len;
        return next;
    }

    public boolean atTarget() {
        return Math.abs(calculateError()) <= positionDeadband;
    }

    public double calculateError(){
        // Signed shortest-path error on a unit circle [-0.5, 0.5]
        // Positive error means "move forward" (increasing position) to reach the target.
        double e = targetPos - pos;

        if (approaching) {
            if (lockedDir > 0 && e < 0) e += 1.0;
            if (lockedDir < 0 && e > 0) e -= 1.0;
        } else {
            if (e > 0.5) e -= 1.0;
            if (e < -0.5) e += 1.0;
        }
        return e;
    }
    public void updatePosition() {
        pos = getPosition();
    }

    public double updatePIDControl() {
        if (!enableControl) {
            // Hold current position (no output) and clear integrators
            integral = 0.0;
            lastErr = 0.0;
            setPower(0.0);
            return 0.0;
        }

        error = calculateError();

        if (Math.abs(error) <= positionDeadband) {
            approaching = false;
        }

        double dt = timer.seconds();
        if (dt <= 0) dt = 0.02;

        integral += error * dt;
        double derivative = (error - lastErr) / dt;
        lastErr = error;

        double out = (error * kP) + (derivative * kD) + (integral * kI) + Math.signum(error) * kF;
        out = Range.clip(out, -MAX_POW, MAX_POW);
        if (!approaching && Math.abs(error) <= positionDeadband) {
            out = Range.clip(out, -0.25, 0.25);
        }

        setPower(out);
        timer.reset();
        return out;
    }

    public void update() {
        updatePosition();
        updatePIDControl();
    }

    public void setPower(double power) {
        cycle1.setPower(power);
        cycle2.setPower(power);
    }

    public double getTargetPos(){
        return targetPos;
    }
    private static int preferredSignFromIndexDelta(int fromIdx, int toIdx, int length) {
        if (length <= 0) return 1;
        int delta = (toIdx - fromIdx) % length;
        if (delta < 0) delta += length;
        if (delta == 0) return 1;
        // If exactly half-turn in index space (even length), pick a consistent direction (+).
        if ((length % 2 == 0) && (delta == length / 2)) return 1;
        return (delta < length / 2) ? 1 : -1;
    }
}
