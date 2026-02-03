package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Math helpers for a continuous, wrap-around cycle wheel / indexer where
 * normalized position is on a unit circle: [0, 1).
 */
public final class CycleWheelMath {

    private CycleWheelMath() { }

    /** Wrap a normalized coordinate into [0, 1). */
    public static double wrap01(double x) {
        double y = x % 1.0;
        if (y < 0.0) y += 1.0;
        return y;
    }

    /**
     * Returns the signed shortest error from {@code current} to {@code target}
     * on a unit circle. Result is in (-0.5, 0.5].
     */
    public static double circleError(double target, double current) {
        double e = wrap01(target) - wrap01(current);
        // Wrap into (-0.5, 0.5]
        if (e > 0.5) e -= 1.0;
        if (e <= -0.5) e += 1.0;
        return e;
    }

    public static double circleAbsError(double target, double current) {
        return Math.abs(circleError(target, current));
    }

    /** Normalizes an analog voltage into [0, 1]. */
    public static double normalizeVoltage(double v, double minV, double maxV) {
        double denom = maxV - minV;
        if (Math.abs(denom) < 1e-9) return 0.0;
        return Range.clip((v - minV) / denom, 0.0, 1.0);
    }

    /** Returns the index whose center is closest to {@code posNorm} on the circle. */
    public static int nearestIndexOnCircle(double[] centers01, double posNorm) {
        if (centers01 == null || centers01.length == 0) return 0;

        int bestIdx = 0;
        double bestErr = Double.POSITIVE_INFINITY;

        for (int i = 0; i < centers01.length; i++) {
            double err = circleAbsError(centers01[i], posNorm);
            if (err < bestErr) {
                bestErr = err;
                bestIdx = i;
            }
        }
        return bestIdx;
    }
}

