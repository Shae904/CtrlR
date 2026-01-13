package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/*
port configs
control hub
motors
0 - fr
1 - fl
2 - br
3 - bl
expansion hub
motors
2 - launch
3 - intake
servos
1 - transfer
4 - cycle

axon max analog feedback wire -> analog input named "cycleAnalog"
*/
@Configurable
public class Robot {

    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotorEx intake, launch;

    public final ServoImplEx transfer, cycle;

    public static LinearOpMode opMode;

    public static double transferOne = 0;
    public static double transferTwo = 0.38;

    // 0..2 cycle indices
    public static double[] cyclePos = {0.055, 0.343, 0.615};
    public int cpos = 0;

    public final Limelight3A limelight;

    public static double cycleTime = 0.9;     // fallback if you ever need it
    public static double outTime = 0.5;
    public static double transferTime = 0.2;

    // distance to goal (updated when tag is seen)
    private double x = 128;

    // ===== aim pid (tx -> rx) tunables =====
    public static double AIM_Kp = 0.016;
    public static double AIM_Ki = 0.0;
    public static double AIM_Kd = 0.0017;
    public static double AIM_Ks = 0.06;
    public static double AIM_DEADBAND = 0.4;

    public static double AIM_OFFSET_RED = 0.0;
    public static double AIM_OFFSET_BLUE = 0.0;

    // ===== cycle analog feedback (axon max) =====
    public AnalogInput cycleAnalog;

    // tune these by printing voltage at setCycle(0) and setCycle(2)
    public static double CYCLE_V_MIN = 0.30; // volts at servoPos ~0.0
    public static double CYCLE_V_MAX = 2.90; // volts at servoPos ~1.0
    public static double CYCLE_V_TOL = 0.05; // volts tolerance = "close enough"

    // how many consecutive reads must be in-tolerance to count as aligned
    public static int CYCLE_STABLE_COUNT = 3;

    // set false in an opmode before new Robot(this) if you don't want camera init
    public static boolean INIT_VISION = true;

    // vision
    public OpenCvWebcam webcam;
    public C920PanelsEOCV.C920Pipeline pipeline;

    public Robot(LinearOpMode opMode) {
        Robot.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        // drivetrain
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");

        fl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        bl.setDirection(Direction.REVERSE);
        br.setDirection(Direction.FORWARD);

        fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        opMode.telemetry.addData("imu initialized", true);
        opMode.telemetry.update();

        // servos
        cycle = (ServoImplEx) hardwareMap.get(Servo.class, "cycle");
        transfer = (ServoImplEx) hardwareMap.get(Servo.class, "transfer");

        // intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // outtake
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(RunMode.RUN_WITHOUT_ENCODER);
        launch.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        launch.setDirection(Direction.REVERSE);

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        // cycle analog (optional but recommended)
        try {
            cycleAnalog = hardwareMap.get(AnalogInput.class, "cycleAnalog");
        } catch (Exception ignored) {
            cycleAnalog = null;
        }

        // vision (optional)
        if (INIT_VISION) {
            startVision();
        }
    }

    public void startVision() {
        if (webcam != null) return;

        HardwareMap hardwareMap = opMode.hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "c920"), cameraMonitorViewId);

        pipeline = new C920PanelsEOCV.C920Pipeline();
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);
        webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        opMode.telemetry.addLine("initializing camera...");
        opMode.telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("camera error", errorCode);
                opMode.telemetry.update();
            }
        });
    }

    public void stopVision() {
        if (webcam == null) return;
        try {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        } catch (Exception ignored) { }
        webcam = null;
        pipeline = null;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    // ===== cycle analog helpers =====

    public boolean hasCycleAnalog() {
        return cycleAnalog != null;
    }

    public double getCycleVoltage() {
        if (cycleAnalog == null) return -1.0;
        return cycleAnalog.getVoltage();
    }

    private double expectedCycleVoltage(double servoPos) {
        // assumes roughly linear mapping (good enough for gating)
        return CYCLE_V_MIN + servoPos * (CYCLE_V_MAX - CYCLE_V_MIN);
    }

    public boolean cycleAtServoPos(double servoPos) {
        if (cycleAnalog == null) return false;
        double v = getCycleVoltage();
        double vt = expectedCycleVoltage(servoPos);
        return Math.abs(v - vt) <= CYCLE_V_TOL;
    }

    public boolean cycleAtIndex(int cycleIndex) {
        if (cycleIndex < 0 || cycleIndex > 2) return false;
        return cycleAtServoPos(cyclePos[cycleIndex]);
    }

    // chamber that fires if you do nothing
    public int getFireSlot() {
        return (cpos + 1) % 3;
    }

    // IMPORTANT: +1 because “next chamber” is the one that fires
    public int cycleIndexForSlotIndex(int slotIndex) {
        return (cpos + slotIndex + 1) % 3;
    }

    // ===== mechanisms =====

    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
    }

    public void setCycle(int pos) {
        cpos = pos;
        cycle.setPosition(cyclePos[pos]);
    }

    public void setLaunch(double power) {
        launch.setPower(power);
    }

    public void transferUp() {
        transfer.setPosition(transferOne);
    }

    public void transferDown() {
        transfer.setPosition(transferTwo);
    }

    // flywheel velocity control
    public double outtake(char color) {
        double Kv = 0.000379;
        double Kp = 0.001;

        double goalHeight = 30;
        double limelightHeight = 10;

        double angle = -1;
        int targetId = (color == 'r') ? 24 : 20;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetId) {
                        double y = fiducial.getTargetYDegrees();
                        angle = Math.toRadians(y + 21);
                        break;
                    }
                }
            }
        }

        if (angle != -1) {
            x = (goalHeight - limelightHeight) / Math.tan(angle) + 6;
        }

        double targetVelo = 112.57 * x * Math.pow(0.9035693 * x - 29, -0.5);
        double ff = Kv * targetVelo;

        double currentVelo = launch.getVelocity();
        double p = Kp * (targetVelo - currentVelo);

        double power = Range.clip(ff + p, -0.2, 1.0);
        launch.setPower(power);

        return targetVelo;
    }
}
