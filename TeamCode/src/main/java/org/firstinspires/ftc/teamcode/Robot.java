// Robot.java
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.teleop.C920PanelsEOCV;
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
    public final DriveSubsystem drive;

    public final IntakeSubsystem intake;
    public final DcMotorEx launch;
    private final ServoImplEx transfer, cycle;
    public final ShooterSubsystem shooter;

    public static LinearOpMode opMode;

    public static double transferOne = 0.0;
    public static double transferTwo = 0.38;

    public static double[] cyclePos = {0.055, 0.343, 0.615};
    public int cpos = 0;

    public final Limelight3A limelight;

    public static double cycleTime = 0.9;     // todo tune
    public static double outTime = 0.5;       // todo tune

    public static double launchMult = 143.4;

    public static double transferTime = 0.2;  // todo tune

    // ===== one-person macro timings (shared) =====
// Used by OnePersonAltRedTeleop fire-test and sort3 macros (time-based; no analog gating)
    public static double FIRE_CYCLE_SETTLE_TIME = 0.5; // wait after setCycle() before feeding
    public static double FIRE_FEED_DELAY = 0.4;        // extra delay before transferUp
    public static double FIRE_FEED_TIME = 0.35;         // transferUp duration (raise if arm barely lifts ball)
    public static double FIRE_DOWN_TIME = 0.40;         // time between shots with transferDown

    // ===== aim pid (tx -> rx) tunables (for sharing / panels, not used inside Robot yet) =====
    public static double AIM_Kp = 0.016;
    public static double AIM_Ki = 0.0;
    public static double AIM_Kd = 0.0017;
    public static double AIM_Ks = 0.06;
    public static double AIM_DEADBAND = 0.4;

    public static double AIM_OFFSET_RED = -4.4;
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
        drive = new DriveSubsystem(fl, fr, bl, br);

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
        intake = new IntakeSubsystem(hardwareMap.get(DcMotorEx.class, "intake"));

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        // outtake
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        shooter = new ShooterSubsystem(launch, transfer, cycle, limelight);

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

    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
    }

    // placeholder so tuner/teleop can call it (pid state is in those opmodes)
    public void resetAim() { }

    // mechanisms
    public void setCycle(int pos) {
        cpos = pos;
        shooter.setCyclePosition(cyclePos[pos]);
    }

    public void setCyclePosition(double position) {
        shooter.setCyclePosition(position);
    }

    public void setLaunch(double power) {
        shooter.setLaunchPower(power);
    }

    public void transferUp() {
        shooter.setTransferPosition(transferOne);
    }

    public void transferDown() {
        shooter.setTransferPosition(transferTwo);
    }

    // motor test helper (optional)
    public void setMotor(int motor, double power) {
        drive.setMotor(motor, power);
    }

    // flywheel velocity control (same math you had)
    public double outtake(char color) {
        return shooter.outtake(color, launchMult);
    }
}
