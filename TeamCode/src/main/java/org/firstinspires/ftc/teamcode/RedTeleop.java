package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@TeleOp(name = "Red Teleop")
public class RedTeleop extends LinearOpMode {

    // vision: c920 + eocv pipeline for 3-slot detection

    private OpenCvWebcam webcam;
    private C920PanelsEOCV.C920Pipeline pipeline;

    private final C920PanelsEOCV.C920Pipeline.SlotState[] prevColors =
            new C920PanelsEOCV.C920Pipeline.SlotState[3];
    private final int[] frameCount = new int[3];

    private boolean autoFire = false;

    // april tag pattern (sorting motif) from limelight (21 / 22 / 23)
    private int pattern = -1;
    // which shot in the 3-step pattern we’re on (0, 1, 2)
    private int patternStage = 0;

    // robot + shooter state
    public static Robot robot;
    private int out = 0;
    private final ElapsedTime shootTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.limelight.start();
        robot = new Robot(this);

        initSlotConfirmation();

        waitForStart();

        // first try at reading pattern
        updatePatternFromLimelight();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY  = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

            // Slow mode
            if (gamepad1.right_bumper) {
                frontLeftPower *= 0.4;
                backLeftPower *= 0.4;
                frontRightPower *= 0.4;
                backRightPower *= 0.4;
            }

            // Intake
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                robot.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);
            }

            if (gamepad2.y) {
                robot.setCycle(0);
            }
            if (gamepad2.b) {
                robot.setCycle(1);
            }
            if (gamepad2.x) {
                robot.setCycle(2);
            }

            // keep pattern updated while tags are in view
            updatePatternFromLimelight();

            // target color for this shot based on pattern + stage
            C920PanelsEOCV.C920Pipeline.SlotState desiredColor =
                    desiredColorFromPattern(pattern, patternStage);

            handleShooting(desiredColor);

            sendTelemetry();
        }

        robot.webcam.stopStreaming();
        robot.webcam.closeCameraDevice();
    }

    private void initSlotConfirmation() {
        for (int i = 0; i < 3; i++) {
            prevColors[i] = C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
            frameCount[i] = 0;
        }
    }

    // ==================== shooting / indexer ====================

    private void handleShooting(C920PanelsEOCV.C920Pipeline.SlotState desiredColor) {
        // manual override
        if (gamepad2.a) {
            autoFire = false;

            if (out == 0) {
                shootTime.reset();
            }

            robot.outtake('r', shootTime.seconds());
            out = 1;
            return;
        }

        // auto firing currently in progress
        if (autoFire) {
            robot.outtake('r', shootTime.seconds());
            out = 1;

            // done with full cycle, stop and bump pattern stage
            if (shootTime.seconds() > 2 * Robot.cycleTime + Robot.outTime) {
                robot.stopOuttake(0);
                out = 0;
                autoFire = false;
                advancePatternStage();
            }
            return;
        }

        // no button and no auto sequence
        if (out == 1) {
            // last loop we were shooting, finish it and advance indexer via robot
            robot.stopOuttake(1);
            out = 0;
        }

        // try to auto fire from chamber if vision says so
        attemptAutoShot(desiredColor);

        // still allow manual indexer moves
    }

    private void attemptAutoShot(C920PanelsEOCV.C920Pipeline.SlotState desiredColor) {
        C920PanelsEOCV.C920Pipeline.SlotState[] states = pipeline.getSlotStates();

        // 3-frame confirmation per slot
        updateSlotConfirmation(states);

        // pick which slot to shoot from
        int chosenSlot = selectSlotToFire(states, desiredColor);

        // no good slot
        if (chosenSlot == -1) return;

        startAutoShotFromSlot(chosenSlot);
    }
    //andrew aburustum is a freaking goober
    // slot confirmation

    private void updateSlotConfirmation(C920PanelsEOCV.C920Pipeline.SlotState[] states) {
        for (int i = 0; i < 3; i++) {
            C920PanelsEOCV.C920Pipeline.SlotState current = states[i];

            if (current != C920PanelsEOCV.C920Pipeline.SlotState.EMPTY) {
                if (current == prevColors[i]) {
                    frameCount[i]++;
                } else {
                    frameCount[i] = 1;
                    prevColors[i] = current;
                }
            } else {
                frameCount[i] = 0;
                prevColors[i] = C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
            }
        }
    }

    // 1) match desired color, 2) else any confirmed non-empty
    private int selectSlotToFire(C920PanelsEOCV.C920Pipeline.SlotState[] states,
                                 C920PanelsEOCV.C920Pipeline.SlotState desiredColor) {

        if (desiredColor != C920PanelsEOCV.C920Pipeline.SlotState.EMPTY) {
            for (int i = 0; i < 3; i++) {
                if (frameCount[i] >= 3 && states[i] == desiredColor) {
                    return i;
                }
            }
        }

        for (int i = 0; i < 3; i++) {
            if (frameCount[i] >= 3 &&
                    states[i] != C920PanelsEOCV.C920Pipeline.SlotState.EMPTY) {
                return i;
            }
        }

        return -1;
    }

    private void startAutoShotFromSlot(int slotIndex) {
        robot.setCycle(slotIndex);
        shootTime.reset();
        robot.outtake('r', shootTime.seconds());
        out = 1;
        autoFire = true;

        // clear so we don't instantly retrigger
        frameCount[slotIndex] = 0;
        prevColors[slotIndex] = C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
    }

    // LIMELIGHT+PATTERN

    private void updatePatternFromLimelight() {
        if (robot.limelight == null) return;

        LLResult result = robot.limelight.getLatestResult();
        if (result == null) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (id >= 21 && id <= 23) {
                // if pattern changes mid-match, reset sequence
                if (pattern != id) {
                    pattern = id;
                    patternStage = 0;
                }
                break;
            }
        }
    }

    // requested orders:
    // 21: g, p, p
    // 22: p, g, p
    // 23: p, p, g
    private C920PanelsEOCV.C920Pipeline.SlotState desiredColorFromPattern(int pattern, int stage) {
        int s = ((stage % 3) + 3) % 3; // clamp to 0–2

        switch (pattern) {
            case 21:
                // g, p, p
                if (s == 0) return C920PanelsEOCV.C920Pipeline.SlotState.GREEN;
                return C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

            case 22:
                // p, g, p
                if (s == 1) return C920PanelsEOCV.C920Pipeline.SlotState.GREEN;
                return C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

            case 23:
                // p, p, g
                if (s == 2) return C920PanelsEOCV.C920Pipeline.SlotState.GREEN;
                return C920PanelsEOCV.C920Pipeline.SlotState.PURPLE;

            default:
                // no valid tag yet
                return C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
        }
    }

    // move to next shot in sequence
    private void advancePatternStage() {
        patternStage = (patternStage + 1) % 3;
    }

    // TELMETRY

    private void sendTelemetry() {
        telemetry.addData("pattern (tag)", pattern);
        telemetry.addData("pattern stage", patternStage);

        C920PanelsEOCV.C920Pipeline.SlotState[] slotStates = pipeline.getSlotStates();
        telemetry.addData("slot 0", slotStates[0]);
        telemetry.addData("slot 1", slotStates[1]);
        telemetry.addData("slot 2", slotStates[2]);

        telemetry.update();
    }
}
