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

    private double spPos = 0.0;
    private Robot robot;

    private double spVel = 0.0;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        timer.reset();

        boolean lastA = false, lastB = false, lastX = false, lastY = false;

        while (opModeIsActive()) {
            // ----- Gamepad helpers -----
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            if (a && !lastA) robot.cycleCW();
            if (b && !lastB) robot.cycleCCW();

            // Capture calibration
            if (x && !lastX) SpinSorter.minV = robot.servoPos.getVoltage();
            if (y && !lastY) SpinSorter.maxV = robot.servoPos.getVoltage();

            lastA = a; lastB = b; lastX = x; lastY = y;

            robot.spinSorter.updatePosition();
            robot.spinSorter.updatePIDControl();

            // ----- Telemetry -----
            telemetry.addData("targetIndex", "%d", robot.spinSorter.getTargetIndex());
            telemetry.addData("targetPos", "%.3f", robot.spinSorter.getTargetPos());
            telemetry.addData("pos", "%.3f", robot.spinSorter.pos);
            telemetry.addData("error (signed)", "%.3f", robot.spinSorter.error);
            telemetry.addData("|error|", "%.3f", Math.abs(robot.spinSorter.error));
            telemetry.addData("spPos/spVel", "%.3f / %.3f", spPos, spVel);
            telemetry.addData("PID", "P %.2f  I %.2f  D %.2f", SpinSorter.kP, SpinSorter.kI, SpinSorter.kD);
            telemetry.addData("voltage", "%.3f", robot.servoPos.getVoltage());
            telemetry.addData("minV/maxV", "%.3f / %.3f", SpinSorter.minV, SpinSorter.maxV);
            telemetry.update();
        }
    }
}
