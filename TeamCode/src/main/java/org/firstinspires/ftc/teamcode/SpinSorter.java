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

    private final ElapsedTime timer = new ElapsedTime();

    private double targetPos = 0.0;
    private int targetIndex = 0;

    private double integral = 0.0;
    private double lastErr = 0.0;

    public double pos = 0;
    public double error = 0;

    public SpinSorter(CRServo cycle1, CRServo cycle2, AnalogInput analog) {
        this.cycle1 = cycle1;
        this.cycle2 = cycle2;
        this.analog = analog;
        resetToCurrent();
    }

    public void resetToCurrent() {
        targetPos = getPosition();
        targetIndex = (int) Math.round((presetPositions.length+1) / 2.0);
        integral = 0.0;
        lastErr = 0.0;
        timer.reset();
    }

    public double normalize(double voltage){
        double denom = maxV - minV;
        return Range.clip((voltage - minV) / denom,0.0,1.0);
    }
    public double getPosition() {
        return normalize(analog.getVoltage());
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
        targetIndex = (targetIndex + 1) % Math.max(1, SpinSorter.presetPositions.length);
        setTargetPos(presetPositions[targetIndex]);
    }
    public void cycleCCW(){
        targetIndex = (targetIndex - 1 + Math.max(1, SpinSorter.presetPositions.length)) % Math.max(1, SpinSorter.presetPositions.length);
        setTargetPos(presetPositions[targetIndex]);
    }
    public void setIndex(int index) {
        setTargetPos(presetPositions[index]);
    }

    public boolean atTarget() {
        double err = Math.abs(targetPos - pos);
        return err <= positionDeadband;
    }

    public double calculateError(){
        double e = Math.abs(targetPos - pos);
        double e2 = Math.abs(e + 1);
        double e3 = Math.abs(e - 1);
        return Math.min(Math.min(e,e2),e3);
    }

    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 0.02;

        if (!enableControl) {
            // Hold current position (no output) and clear integrators
            integral = 0.0;
            lastErr = 0.0;
            setPower(0.0);
            return;
        }
        pos = getPosition();

        double error = calculateError();

        integral += error * dt;
        double derivative = (error - lastErr) / dt;

        lastErr = error;

        double out = (error * kP) + (derivative * kD) + (integral * kI) + Math.signum(error) * kF;

        out = Range.clip(out, -MAX_POW, MAX_POW);
        timer.reset();
        setPower(out);
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
