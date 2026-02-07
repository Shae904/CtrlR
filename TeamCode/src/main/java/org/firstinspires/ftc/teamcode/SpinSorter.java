package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
    // Tunables
    // =========================

    // Analog calibration
    public static double minV = 0.008;
    public static double maxV = 3.294;
    public static boolean invertSensor = true;

    /**
     * Preset "stop" positions on the unit circle [0,1).
     * Keep these sorted (increasing), with the last entry near 1.0 if you use wrap-around.
     */
    public static double[] presetPositions = {
            0.065,
            0.232,
            0.405,
            0.566,
            0.737,
            0.906
    };

    // Enable controller output
    public static boolean enableControl = true;

    // PF gains (normalized position -> power)
    // Tune these on the Dashboard
    public static double kP = 1;
    public static double kF = 0.015;  // static friction compensation (sign(error) * kF)

    public static double MAX_POW = 1.0;
    public static double LOW_POW = 0.5;
    public static double positionDeadband = 0.003;  // target "in-band" threshold

    // =========================
    // Hardware
    // =========================

    private final CRServo cycle1;
    private final CRServo cycle2;
    private final AnalogInput analog;

    // =========================
    // Internal controller state
    // =========================

    private double targetPos = 0.0;   // desired position [0,1)
    private int targetIndex = 0;      // index into presetPositions[]

    private double pos = 0.0;         // current position [0,1)
    private double error = 0.0;       // current signed error (shortest path)

    // For telemetry only; not used for direction-lock logic anymore
    private boolean approaching = false;
    private int lockedDir = 1;        // sign of last error

    // =========================
    // Constructor
    // =========================

    public SpinSorter(CRServo cycle1, CRServo cycle2, AnalogInput analog) {
        this.cycle1 = cycle1;
        this.cycle2 = cycle2;
        this.analog = analog;

        // Initialize position & snap target to nearest preset so A/B are consistent
        updatePosition();
        this.targetIndex = nearestPresetIndex(this.pos);
        this.targetPos = presetPositions[targetIndex];
    }

    // =========================
    // Helpers
    // =========================

    public static int midPresetIndex() {
        if (presetPositions == null || presetPositions.length == 0) return 0;
        return presetPositions.length / 2;
    }

    /** Wraps any value to [-0.5, 0.5] (shortest-path on unit circle). */
    private static double wrapShortest(double e) {
        if (e > 0.5) e -= 1.0;
        if (e < -0.5) e += 1.0;
        return e;
    }

    private static double circularDistance(double a, double b) {
        return Math.abs(wrapShortest(a - b));
    }

    /** Find the nearest preset index to a given position. */
    private static int nearestPresetIndex(double p) {
        if (presetPositions == null || presetPositions.length == 0) return 0;

        int best = 0;
        double bestDist = Double.POSITIVE_INFINITY;

        for (int i = 0; i < presetPositions.length; i++) {
            double d = circularDistance(p, presetPositions[i]);
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        return best;
    }

    public double normalize(double voltage){
        double denom = maxV - minV;
        if (Math.abs(denom) < 1e-9) return 0.0;
        return Range.clip((voltage - minV) / denom, 0.0, 1.0);
    }

    // =========================
    // Getters for telemetry
    // =========================

    public double getError() {
        return error;
    }

    public int getTargetIndex() {
        return targetIndex;
    }

    public double getTargetPos(){
        return this.targetPos;
    }

    public double getCurrentPos() {
        return this.pos;
    }

    public int getLockedDir() {
        return lockedDir;
    }

    public boolean isApproaching() {
        return approaching;
    }

    // =========================
    // Position + target management
    // =========================

    public void updatePosition() {
        double p = normalize(analog.getVoltage());
        if (invertSensor) p = 1.0 - p;

        // keep in [0,1)
        if (p >= 1.0) p -= 1.0;
        if (p < 0.0)  p += 1.0;

        this.pos = p;
    }

    public void setTargetPos(double target) {
        // target is assumed to already be in [0,1)
        if (target != targetPos) {
            targetPos = target;
            approaching = true;
        }
    }

    public void setIndex(int index) {
        if (presetPositions == null || presetPositions.length == 0) return;
        int len = presetPositions.length;
        int idx = index % len;
        if (idx < 0) idx += len;

        targetIndex = idx;
        setTargetPos(presetPositions[idx]);
    }

    /** Move to next preset in the list: 0→1→2→...→N-1→0. */
    public void cycleCW() {
        if (presetPositions == null || presetPositions.length == 0) return;
        int len = presetPositions.length;

        targetIndex = (targetIndex + 1) % len;
        setTargetPos(presetPositions[targetIndex]);
    }

    /** Move to previous preset in the list: 0→N-1→...→2→1→0. */
    public void cycleCCW() {
        if (presetPositions == null || presetPositions.length == 0) return;
        int len = presetPositions.length;

        targetIndex = (targetIndex - 1 + len) % len;
        setTargetPos(presetPositions[targetIndex]);
    }

    public boolean atTarget() {
        return atTarget(positionDeadband);
    }

    public boolean atTarget(double deadband) {
        // always use shortest-path error for this
        double e = calculateError();
        return Math.abs(e) <= deadband;
    }

    // =========================
    // Control
    // =========================

    public double calculateError(){
        // shortest path from pos to target on unit circle
        double e = targetPos - pos;
        e = wrapShortest(e);
        return e;
    }

    public double updatePIDControl() {
        if (!enableControl) {
            this.setPower(0.0);
            return 0.0;
        }

        this.error = calculateError();
        this.lockedDir = (error >= 0) ? 1 : -1;   // for telemetry only
        this.approaching = Math.abs(error) > positionDeadband;

        // Treat deadband as "stop" condition
        /*if (Math.abs(this.error) <= positionDeadband) {
            this.setPower(0.0);
            return 0.0;
        }*/

        // PF control: proportional + static friction compensation
        double out = (error * kP) + Math.signum(error) * kF;
        out = Range.clip(out, -MAX_POW, MAX_POW);
        if(!approaching && Math.abs(error) <= positionDeadband){
            out = Range.clip(out, -LOW_POW,LOW_POW);
        }

        setPower(out);
        return out;
    }

    public void update() {
        updatePosition();
        updatePIDControl();
    }

    // =========================
    // Output
    // =========================

    public void setPower(double power) {
        this.cycle1.setPower(power);
        this.cycle2.setPower(power);
    }
}
