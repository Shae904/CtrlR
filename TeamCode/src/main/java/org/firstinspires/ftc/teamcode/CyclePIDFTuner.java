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

    // Analog calibration
    public static double minV = 0.10;
    public static double maxV = 3.20;
    public static boolean isWraparound = true;

    // Target positions (normalized 0..1)
    public static double[] positions = {
            0.096,
            0.260,
            0.430,
            0.595,
            0.764,
            0.936
    };
    public static int targetIndex = 0;

    // Enable controller output
    public static boolean enableControl = true;

    // Motion profile limits
    public static double maxVel = 1.5;      // units/sec
    public static double maxAccel = 4.0;    // units/sec^2

    // PID gains
    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.15;

    // Feedforward
    public static double kS = 0.0;  // static friction
    public static double kV = 0.0;  // velocity FF

    // Output limits
    public static double maxPower = 1.0;
    public static double outputDeadband = 0.0;
    public static double positionDeadband = 0.01;
    public static double iClamp = 0.25;

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
        AnalogInput servoPos = hardwareMap.get(AnalogInput.class, "servoAnalog");

        ElapsedTime timer = new ElapsedTime();

        // Initialize setpoint to current position
        double voltage = servoPos.getVoltage();
        double pos = normalize(voltage);
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
            pos = normalize(voltage);

            // ----- Gamepad helpers -----
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            if (a && !lastA) targetIndex = (targetIndex + 1) % positions.length;
            if (b && !lastB) targetIndex = (targetIndex - 1 + positions.length) % positions.length;

            // Capture calibration
            if (x && !lastX) minV = voltage;
            if (y && !lastY) maxV = voltage;

            lastA = a; lastB = b; lastX = x; lastY = y;

            // ----- Target -----
            int idx = Range.clip(targetIndex, 0, positions.length - 1);
            double target = Range.clip(positions[idx], 0.0, 1.0);

            // ----- Motion profile -----
            if (enableControl) {
                updateProfile(target, dt);
            } else {
                spPos = pos;
                spVel = 0.0;
                integral = 0.0;
                lastErr = 0.0;
            }

            // ----- Error -----
            double err = positionError(spPos, pos);
            if (Math.abs(err) < positionDeadband) {
                err = 0.0;
            }

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

            // ----- Clamp & deadband -----
            out = Range.clip(out, -maxPower, maxPower);
            if (Math.abs(out) < outputDeadband) out = 0.0;

            // =================================================
            // EXACT SAME POWER TO BOTH SERVOS — ALWAYS
            // =================================================
            cycle1.setPower(out);
            cycle2.setPower(out);

            // ----- Telemetry -----
            telemetry.addData("index", "%d / %d", idx, positions.length - 1);
            telemetry.addData("target", "%.3f", target);
            telemetry.addData("pos", "%.3f", pos);
            telemetry.addData("spPos/spVel", "%.3f / %.3f", spPos, spVel);
            telemetry.addData("err", "%.3f", err);
            telemetry.addData("out", "%.3f", out);
            telemetry.addData("PID", "P %.2f  I %.2f  D %.2f", kP, kI, kD);
            telemetry.addData("FF", "kS %.2f  kV %.2f", kS, kV);
            telemetry.addData("V", "%.3f", voltage);
            telemetry.addData("minV/maxV", "%.3f / %.3f", minV, maxV);
            telemetry.update();
        }
    }

    // =========================
    // Helper functions
    // =========================

    private double normalize(double v) {
        double denom = maxV - minV;
        if (Math.abs(denom) < 1e-6) return 0.0;
        return Range.clip((v - minV) / denom, 0.0, 1.0);
    }

    private double positionError(double target, double current) {
        double e = target - current;
        if (!isWraparound) return e;

        double e2 = e - 1.0;
        double e3 = e + 1.0;

        if (Math.abs(e2) < Math.abs(e)) return e2;
        if (Math.abs(e3) < Math.abs(e)) return e3;
        return e;
    }

    private void updateProfile(double target, double dt) {
        double e = positionError(target, spPos);

        double desiredVel = Range.clip(e / Math.max(dt, 0.02), -maxVel, maxVel);

        double dvMax = maxAccel * dt;
        spVel += Range.clip(desiredVel - spVel, -dvMax, dvMax);
        spVel = Range.clip(spVel, -maxVel, maxVel);

        spPos += spVel * dt;

        if (!isWraparound) {
            spPos = Range.clip(spPos, 0.0, 1.0);
        } else {
            spPos = (spPos % 1.0 + 1.0) % 1.0;
        }
    }
}
