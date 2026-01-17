package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;            // ✅ needed
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

@Configurable
@TeleOp(name = "C920 EasyOpenCV (Panels)", group = "Vision")
public class C920PanelsEOCV extends LinearOpMode {

    private OpenCvWebcam webcam;
    private TelemetryManager telemetryM;

    // sliders in panels (debug only)
    public static int lowerH = 0;
    public static int lowerS = 0;
    public static int lowerV = 0;

    public static int upperH = 180;
    public static int upperS = 255;
    public static int upperV = 255;

    // ===== preprocessing / classification (Panels tunables) =====
    // Choose ONE mode:
    public static boolean useHsvClassifier = true;  // legacy HSV masks
    public static boolean useLabClassifier = false; // ✅ robust to lighting changes (recommended)

    public static int blurKernel = 5;      // must be odd >= 1
    public static double blurSigma = 0.0;

    // Used in HSV mask mode only (legacy logic).
    public static int minPixelsForBall = 1350;

    // HSV thresholds (tunable in Panels, used when useHsvClassifier == true)
    public static int hsvGreenLowerH = 75;
    public static int hsvGreenLowerS = 105;
    public static int hsvGreenLowerV = 150;
    public static int hsvGreenUpperH = 93;
    public static int hsvGreenUpperS = 255;
    public static int hsvGreenUpperV = 255;

    public static int hsvPurpleLowerH = 130;
    public static int hsvPurpleLowerS = 60;
    public static int hsvPurpleLowerV = 60;
    public static int hsvPurpleUpperH = 160;
    public static int hsvPurpleUpperS = 255;
    public static int hsvPurpleUpperV = 255;

    // Blurred BGR mean classifier (tunable in Panels, used when HSV/LAB are false)
    public static double bgrMinBrightness = 45.0;

    public static double bgrGreenDominance = 25.0;
    public static double bgrGreenMinG = 80.0;

    public static double bgrPurpleDominance = 20.0;
    public static double bgrPurpleRbTolerance = 70.0;
    public static double bgrPurpleMinR = 60.0;
    public static double bgrPurpleMinB = 60.0;

    // ===== ✅ LAB mean classifier (tunable in Panels, used when useLabClassifier == true) =====
    // OpenCV 8-bit Lab: L,a,b in [0..255], neutral gray is ~ (a=128,b=128)
    public static double labMinL = 35.0;           // reject dark noise
    public static double labMinChroma = 18.0;      // reject gray/white floor

    // GREEN: "a" tends lower (more green), b often higher (yellow-ish)
    public static double labGreenAMax = 120.0;
    public static double labGreenBMin = 135.0;

    // PURPLE: "a" tends higher (magenta), b often lower (bluish)
    public static double labPurpleAMin = 145.0;
    public static double labPurpleBMax = 130.0;

    private final C920Pipeline pipeline = new C920Pipeline();

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "c920"),
                cameraMonitorViewId
        );

        webcam.setPipeline(pipeline);
        PanelsCameraStream.INSTANCE.startStream(pipeline, 30);

        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        telemetry.addLine("loading...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.debug("Frame count: " + pipeline.getFrameCount());

            String mode = useHsvClassifier ? "HSV" : (useLabClassifier ? "LAB" : "BGR");
            telemetryM.debug("Mode: " + mode);

            if (useHsvClassifier) {
                telemetryM.debug("Avg H: " + pipeline.getLastH());
                telemetryM.debug("Avg S: " + pipeline.getLastS());
                telemetryM.debug("Avg V: " + pipeline.getLastV());
            } else if (useLabClassifier) {
                telemetryM.debug("Avg L: " + pipeline.getLastH());
                telemetryM.debug("Avg a: " + pipeline.getLastS());
                telemetryM.debug("Avg b: " + pipeline.getLastV());
            } else {
                telemetryM.debug("Avg B: " + pipeline.getLastH());
                telemetryM.debug("Avg G: " + pipeline.getLastS());
                telemetryM.debug("Avg R: " + pipeline.getLastV());
            }

            C920Pipeline.SlotState[] states = pipeline.getSlotStates();
            telemetryM.debug("Slot 1: " + states[0]);
            telemetryM.debug("Slot 2: " + states[1]);
            telemetryM.debug("Slot 3: " + states[2]);

            telemetryM.update(telemetry);
            sleep(20);
        }

        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        PanelsCameraStream.INSTANCE.stopStream();
    }

    public static class C920Pipeline extends OpenCvPipeline implements CameraStreamSource {
        private final Mat bgr = new Mat();
        private final Mat blurredBgr = new Mat();
        private final Mat hsv = new Mat();
        private final Mat lab = new Mat(); // ✅ LAB buffer
        private final Mat mask = new Mat();
        private final Mat maskGreen = new Mat();
        private final Mat maskPurple = new Mat();

        // for triangle masking
        private final Mat slotMask = new Mat();
        private final Mat slotGreen = new Mat();
        private final Mat slotPurple = new Mat();

        private boolean masksInitialized = false;

        private long frameCount = 0;
        private double lastH = 0, lastS = 0, lastV = 0;

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888));

        public enum SlotState {
            EMPTY,
            GREEN,
            PURPLE
        }

        // slot 0 triangle points:
        // (335, 400), (570, 480), (335, 480)
        private final Point[] tri0Points = new Point[] {
                new Point(335, 400),
                new Point(570, 480),
                new Point(335, 480)
        };
        private final MatOfPoint tri0Mat = new MatOfPoint(tri0Points);

        // slot 1 & 2 rectangles
        private final Rect[] slotRects = new Rect[] {
                null,
                new Rect(420, 420, 220, 60),  // slot 1
                new Rect(330, 70, 230, 70)    // slot 2
        };

        private final SlotState[] slotStates = new SlotState[] {
                SlotState.EMPTY, SlotState.EMPTY, SlotState.EMPTY
        };

        @Override
        public Mat processFrame(Mat input) {
            frameCount++;

            // init masks once we know input size
            if (!masksInitialized) {
                slotMask.create(input.rows(), input.cols(), CvType.CV_8UC1);
                masksInitialized = true;
            }

            Imgproc.cvtColor(input, bgr, Imgproc.COLOR_RGBA2BGR);

            int k = C920PanelsEOCV.blurKernel;
            if (k < 1) k = 1;
            if ((k % 2) == 0) k += 1;

            Imgproc.GaussianBlur(bgr, blurredBgr, new Size(k, k), C920PanelsEOCV.blurSigma);

            // ----- MODE SETUP -----
            if (C920PanelsEOCV.useHsvClassifier) {
                Imgproc.cvtColor(blurredBgr, hsv, Imgproc.COLOR_BGR2HSV);

                Scalar lower = new Scalar(lowerH, lowerS, lowerV);
                Scalar upper = new Scalar(upperH, upperS, upperV);
                Core.inRange(hsv, lower, upper, mask);

                Scalar mean = Core.mean(hsv, mask);
                lastH = mean.val[0];
                lastS = mean.val[1];
                lastV = mean.val[2];

                Scalar greenLower = new Scalar(hsvGreenLowerH, hsvGreenLowerS, hsvGreenLowerV);
                Scalar greenUpper = new Scalar(hsvGreenUpperH, hsvGreenUpperS, hsvGreenUpperV);
                Scalar purpleLower = new Scalar(hsvPurpleLowerH, hsvPurpleLowerS, hsvPurpleLowerV);
                Scalar purpleUpper = new Scalar(hsvPurpleUpperH, hsvPurpleUpperS, hsvPurpleUpperV);

                Core.inRange(hsv, greenLower, greenUpper, maskGreen);
                Core.inRange(hsv, purpleLower, purpleUpper, maskPurple);

            } else if (C920PanelsEOCV.useLabClassifier) {
                // ✅ LAB conversion
                Imgproc.cvtColor(blurredBgr, lab, Imgproc.COLOR_BGR2Lab);

                // For panels telemetry, show mean LAB across full frame
                Scalar mean = Core.mean(lab);
                lastH = mean.val[0]; // L
                lastS = mean.val[1]; // a
                lastV = mean.val[2]; // b

            } else {
                // BGR mean mode
                Scalar mean = Core.mean(blurredBgr);
                lastH = mean.val[0]; // B
                lastS = mean.val[1]; // G
                lastV = mean.val[2]; // R
            }

            // ===== slot 0 triangle =====
            slotMask.setTo(new Scalar(0));
            Imgproc.fillConvexPoly(slotMask, tri0Mat, new Scalar(255));

            SlotState state0 = SlotState.EMPTY;

            if (C920PanelsEOCV.useHsvClassifier) {
                Core.bitwise_and(maskGreen, slotMask, slotGreen);
                Core.bitwise_and(maskPurple, slotMask, slotPurple);

                int greenCount0 = Core.countNonZero(slotGreen);
                int purpleCount0 = Core.countNonZero(slotPurple);

                if (greenCount0 > purpleCount0 && greenCount0 > C920PanelsEOCV.minPixelsForBall) {
                    state0 = SlotState.GREEN;
                } else if (purpleCount0 > greenCount0 && purpleCount0 > C920PanelsEOCV.minPixelsForBall) {
                    state0 = SlotState.PURPLE;
                }

            } else if (C920PanelsEOCV.useLabClassifier) {
                Scalar meanLab = Core.mean(lab, slotMask);
                state0 = classifyByLabMean(meanLab);

            } else {
                Scalar meanBgr = Core.mean(blurredBgr, slotMask);
                state0 = classifyByBgrMean(meanBgr);
            }

            slotStates[0] = state0;

            // draw triangle
            Scalar triColor;
            switch (state0) {
                case GREEN:  triColor = new Scalar(0, 255, 0); break;
                case PURPLE: triColor = new Scalar(255, 0, 255); break;
                default:     triColor = new Scalar(255, 255, 255);
            }
            Imgproc.polylines(input, Collections.singletonList(tri0Mat), true, triColor, 2);

            // ===== slot 1 & 2 rectangles =====
            for (int i = 1; i < slotRects.length; i++) {
                Rect r = slotRects[i];
                if (r == null) {
                    slotStates[i] = SlotState.EMPTY;
                    continue;
                }

                Rect bounded = new Rect(
                        Math.max(0, r.x),
                        Math.max(0, r.y),
                        Math.min(r.width,  Math.max(0, input.cols() - r.x)),
                        Math.min(r.height, Math.max(0, input.rows() - r.y))
                );

                if (bounded.width <= 0 || bounded.height <= 0) {
                    slotStates[i] = SlotState.EMPTY;
                    continue;
                }

                SlotState state = SlotState.EMPTY;

                if (C920PanelsEOCV.useHsvClassifier) {
                    Scalar greenLower = new Scalar(hsvGreenLowerH, hsvGreenLowerS, hsvGreenLowerV);
                    Scalar greenUpper = new Scalar(hsvGreenUpperH, hsvGreenUpperS, hsvGreenUpperV);
                    Scalar purpleLower = new Scalar(hsvPurpleLowerH, hsvPurpleLowerS, hsvPurpleLowerV);
                    Scalar purpleUpper = new Scalar(hsvPurpleUpperH, hsvPurpleUpperS, hsvPurpleUpperV);

                    Mat slotHSV = hsv.submat(bounded);

                    Core.inRange(slotHSV, greenLower, greenUpper, maskGreen);
                    int greenCount = Core.countNonZero(maskGreen);

                    Core.inRange(slotHSV, purpleLower, purpleUpper, maskPurple);
                    int purpleCount = Core.countNonZero(maskPurple);

                    if (greenCount > purpleCount && greenCount > C920PanelsEOCV.minPixelsForBall) {
                        state = SlotState.GREEN;
                    } else if (purpleCount > greenCount && purpleCount > C920PanelsEOCV.minPixelsForBall) {
                        state = SlotState.PURPLE;
                    }

                    slotHSV.release();

                } else if (C920PanelsEOCV.useLabClassifier) {
                    Mat slotLab = lab.submat(bounded);
                    Scalar meanLab = Core.mean(slotLab);
                    state = classifyByLabMean(meanLab);
                    slotLab.release();

                } else {
                    Mat slotBgr = blurredBgr.submat(bounded);
                    Scalar meanBgr = Core.mean(slotBgr);
                    state = classifyByBgrMean(meanBgr);
                    slotBgr.release();
                }

                slotStates[i] = state;

                Scalar boxColor;
                switch (state) {
                    case GREEN:  boxColor = new Scalar(0, 255, 0); break;
                    case PURPLE: boxColor = new Scalar(255, 0, 255); break;
                    default:     boxColor = new Scalar(255, 255, 255);
                }
                Imgproc.rectangle(input, bounded, boxColor, 2);
            }

            // update bitmap for Panels camera stream
            try {
                if (!input.empty()) {
                    Bitmap b = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(input, b);
                    lastFrame.set(b);
                }
            } catch (Exception ignored) {}

            return input;
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        @Override
        public void onDrawFrame(Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {
            // overlays done in processFrame
        }

        public long getFrameCount() { return frameCount; }
        public double getLastH() { return lastH; }
        public double getLastS() { return lastS; }
        public double getLastV() { return lastV; }

        public SlotState[] getSlotStates() { return slotStates; }

        private SlotState classifyByBgrMean(Scalar meanBgr) {
            double b = meanBgr.val[0];
            double g = meanBgr.val[1];
            double r = meanBgr.val[2];

            double brightness = (r + g + b) / 3.0;
            if (brightness < C920PanelsEOCV.bgrMinBrightness) return SlotState.EMPTY;

            double maxRb = Math.max(r, b);
            if ((g - maxRb) >= C920PanelsEOCV.bgrGreenDominance && g >= C920PanelsEOCV.bgrGreenMinG) {
                return SlotState.GREEN;
            }

            double avgRb = (r + b) / 2.0;
            if ((avgRb - g) >= C920PanelsEOCV.bgrPurpleDominance
                    && Math.abs(r - b) <= C920PanelsEOCV.bgrPurpleRbTolerance
                    && r >= C920PanelsEOCV.bgrPurpleMinR
                    && b >= C920PanelsEOCV.bgrPurpleMinB) {
                return SlotState.PURPLE;
            }

            return SlotState.EMPTY;
        }

        // ✅ LAB classifier
        private SlotState classifyByLabMean(Scalar meanLab) {
            double L = meanLab.val[0];
            double a = meanLab.val[1];
            double b = meanLab.val[2];

            double da = a - 128.0;
            double db = b - 128.0;
            double chroma = Math.hypot(da, db);

            if (L < C920PanelsEOCV.labMinL || chroma < C920PanelsEOCV.labMinChroma) {
                return SlotState.EMPTY;
            }

            if (a <= C920PanelsEOCV.labGreenAMax && b >= C920PanelsEOCV.labGreenBMin) {
                return SlotState.GREEN;
            }

            if (a >= C920PanelsEOCV.labPurpleAMin && b <= C920PanelsEOCV.labPurpleBMax) {
                return SlotState.PURPLE;
            }

            return SlotState.EMPTY;
        }
    }
}
