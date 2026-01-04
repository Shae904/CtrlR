package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
*/
@Configurable
public class Robot {

    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotorEx intake, launch;

    public static double transferOne = 0;
    public static double transferTwo = 0.38;

    public final ServoImplEx transfer, cycle;

    public static LinearOpMode opMode;

    public static double[] cyclePos = {0.055, 0.343, 0.615};
    public int cpos = 0;

    public final Limelight3A limelight;

    public static double cycleTime = 0.9;     // todo tune
    public static double outTime = 0.5;       // todo tune
    public static double transferTime = 0.2;  // todo tune

    // distance to goal (inches-ish), updated by limelight y angle when tag is seen
    private double x = 128;

    // ===== aim pid (tx -> rx) tunables =====
    public static double AIM_Kp = 0.016;
    public static double AIM_Ki = 0.0;
    public static double AIM_Kd = 0.0017;
    public static double AIM_Ks = 0.06;
    public static double AIM_DEADBAND = 0.4;

    public static double AIM_OFFSET_RED = 0.0;
    public static double AIM_OFFSET_BLUE = 0.0;

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

        // vision (optional)
        if (INIT_VISION) {
            startVision();
        }
    }

    public void startVision() {
        if (webcam != null) return; // already started

        HardwareMap hardwareMap = opMode.hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "c920"), cameraMonitorViewId);

        pipeline = new C920PanelsEOCV.C920Pipeline();
        webcam.setPipeline(pipeline);

        // gpu view is optional
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

    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
    }

    // placeholder so aimpidtunerteleop can compile
    public void resetAim() {
        // if you move pid state into robot later, clear it here
    }

    public void setCycle(int pos) {
        cpos = pos;
        cycle.setPosition(cyclePos[pos]);
    }

    // this is for motor test
    public void setMotor(int motor, double power) {
        if (motor == 0) {
            fl.setPower(power);
        } else if (motor == 1) {
            fr.setPower(power);
        } else if (motor == 2) {
            bl.setPower(power);
        } else if (motor == 3) {
            br.setPower(power);
        }
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

    public double outtake(char color) {
        double Kv = 0.000379;
        double Kp = 0.001;

        double goalHeight = 30;
        double limelightHeight = 10;

        double angle = -1;

        int targetId = (color == 'r') ? 24 : 20;

        // limelight can return null / invalid sometimes. don't crash.
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    double y = fiducial.getTargetYDegrees();
                    if (id == targetId) {
                        angle = Math.toRadians(y + 21);
                        // 21 is limelight facing angle
                        break;
                    }
                }
            }
        }

        if (angle != -1) {
            x = (goalHeight - limelightHeight) / Math.tan(angle) + 6;
            // 6 added to account for distance between limelight and shooter
        }
        // if no tag, keep last x (don't jump)

        double targetVelo = 112.57 * x * Math.pow(0.9035693 * x - 29, -0.5);
        double ff = Kv * targetVelo;

        double currentVelo = launch.getVelocity();
        double p = Kp * (targetVelo - currentVelo);

        double power = Range.clip(ff + p, -0.2, 1.0);
        launch.setPower(power);

        return targetVelo;
    }
}
