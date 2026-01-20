package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

import java.text.SimpleDateFormat;
import java.util.Collections;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

@Configurable
@TeleOp(name = "C920 EasyOpenCV (Panels)", group = "Vision")
public class C920PanelsEOCV extends LinearOpMode {

    private OpenCvWebcam webcam;
    private TelemetryManager telemetryM;

    // ===== Debug HSV sliders (optional) =====
    public static int lowerH = 0, lowerS = 0, lowerV = 0;
    public static int upperH = 180, upperS = 255, upperV = 255;

    // ===== HSV thresholds for your neon balls =====
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

    public static int minPixelsForBall = 1350;

    // ===== Optional: TFLite ROI classifier =====
    // Put .tflite in TeamCode/src/main/assets/
    public static boolean useTfliteClassifier = false;
    public static String tfliteModelFile = "roi_slot_classifier.tflite";
    public static int tfliteInputSize = 48;
    public static boolean tfliteUseFloatInput = true;
    public static float tfliteMinConfidence = 0.55f;
    public static int inferEveryNFrames = 2;

    // ===== Dataset capture =====
    // Saves crops to /sdcard/FIRST/roi_dataset/<label>/slotX_*.png
    public static boolean datasetCaptureEnabled = false;
    // captureEveryNFrames is now ignored; capture is manual via shutter
    public static int captureEveryNFrames = 3;
    public static int captureImageSize = 48;

    // ===== Auto label from HSV + per-ROI manual override =====
    public static boolean autoLabelFromHSV = true;

    // 0=triangle, 1=bottom rect, 2=top rect (matches slotStates indexing)
    public static int manualSelectedRoi = 0;

    public static boolean manualOverrideSlot0 = false;
    public static boolean manualOverrideSlot1 = false;
    public static boolean manualOverrideSlot2 = false;

    // 0=EMPTY,1=GREEN,2=PURPLE
    public static int manualLabelSlot0 = 0;
    public static int manualLabelSlot1 = 0;
    public static int manualLabelSlot2 = 0;

    private C920Pipeline pipeline;

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "c920"),
                cameraMonitorViewId
        );

        pipeline = new C920Pipeline(hardwareMap.appContext, hardwareMap.appContext.getAssets());
        webcam.setPipeline(pipeline);

        PanelsCameraStream.INSTANCE.startStream(pipeline, 30);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        telemetry.addLine("loading...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        // Debounce
        boolean prevY = false;
        boolean prevLB = false;
        boolean prevLeft = false;
        boolean prevRight = false;
        boolean prevUp = false;
        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;
        boolean prevRB = false;

        waitForStart();

        while (opModeIsActive()) {
            // ===== Toggle capture mode (Y) =====
            boolean y = gamepad1.y;
            if (y && !prevY) datasetCaptureEnabled = !datasetCaptureEnabled;
            prevY = y;

            // ===== ROI selection =====
            // D-pad UP: jump straight to triangle (slot 0)
            boolean up = gamepad1.dpad_up;
            if (up && !prevUp) manualSelectedRoi = 0;
            prevUp = up;

            // D-pad LEFT/RIGHT: cycle among 0,1,2
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            if (left && !prevLeft) manualSelectedRoi = Math.max(0, manualSelectedRoi - 1);
            if (right && !prevRight) manualSelectedRoi = Math.min(2, manualSelectedRoi + 1);
            prevLeft = left;
            prevRight = right;

            // ===== Manual override toggle for selected ROI (LB) =====
            boolean lb = gamepad1.left_bumper;
            if (lb && !prevLB) toggleManualOverride(manualSelectedRoi);
            prevLB = lb;

            // ===== Manual label set (A/B/X) for selected ROI =====
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            if (a && !prevA) setManualLabel(manualSelectedRoi, 0); // EMPTY
            if (b && !prevB) setManualLabel(manualSelectedRoi, 1); // GREEN
            if (x && !prevX) setManualLabel(manualSelectedRoi, 2); // PURPLE
            prevA = a;
            prevB = b;
            prevX = x;

            // ===== Manual shutter (RB) =====
            // When datasetCaptureEnabled is true, pressing RB once
            // requests a one-shot capture of all 3 ROIs on the next frame.
            boolean rb = gamepad1.right_bumper;
            if (rb && !prevRB && datasetCaptureEnabled) {
                pipeline.requestOneShotCapture();
            }
            prevRB = rb;

            // ===== Telemetry =====
            telemetryM.debug("Capture", datasetCaptureEnabled ? "ARMED" : "OFF");
            telemetryM.debug("Saved", pipeline.getSavedCount());
            telemetryM.debug("Mode", useTfliteClassifier ? "TFLITE" : "HSV");
            telemetryM.debug("Auto-label", autoLabelFromHSV ? "HSV" : "MANUAL ONLY");

            telemetryM.debug("Manual ROI", manualSelectedRoi);
            telemetryM.debug("Override S0", manualOverrideSlot0 + " label=" + labelNameFromId(manualLabelSlot0));
            telemetryM.debug("Override S1", manualOverrideSlot1 + " label=" + labelNameFromId(manualLabelSlot1));
            telemetryM.debug("Override S2", manualOverrideSlot2 + " label=" + labelNameFromId(manualLabelSlot2));

            C920Pipeline.SlotState[] states = pipeline.getSlotStates();
            telemetryM.debug("Slot0", states[0].name());
            telemetryM.debug("Slot1", states[1].name());
            telemetryM.debug("Slot2", states[2].name());

            telemetryM.update(telemetry);
            sleep(20);
        }

        if (pipeline != null) pipeline.close();
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
        PanelsCameraStream.INSTANCE.stopStream();
    }

    private static void toggleManualOverride(int roi) {
        if (roi == 0) manualOverrideSlot0 = !manualOverrideSlot0;
        else if (roi == 1) manualOverrideSlot1 = !manualOverrideSlot1;
        else manualOverrideSlot2 = !manualOverrideSlot2;
    }

    private static void setManualLabel(int roi, int label) {
        if (roi == 0) manualLabelSlot0 = label;
        else if (roi == 1) manualLabelSlot1 = label;
        else manualLabelSlot2 = label;
    }

    private static String labelNameFromId(int id) {
        if (id == 1) return "GREEN";
        if (id == 2) return "PURPLE";
        return "EMPTY";
    }

    // =====================================================================
    // Pipeline
    // =====================================================================
    public static class C920Pipeline extends OpenCvPipeline implements CameraStreamSource {
        private final Context context;
        private final AssetManager assets;

        // Mats
        private final Mat rgb = new Mat();
        private final Mat hsv = new Mat();
        private final Mat maskDebug = new Mat();
        private final Mat maskGreen = new Mat();
        private final Mat maskPurple = new Mat();

        private final Mat slotMask = new Mat();
        private final Mat slotGreen = new Mat();
        private final Mat slotPurple = new Mat();

        // TFLite
        private Interpreter interpreter;
        private ByteBuffer inputBuffer;
        private final float[][] output = new float[1][3];

        // ROI resize/capture mats
        private final Mat roiRgb = new Mat();
        private final Mat roiResized = new Mat();

        private boolean masksInitialized = false;
        private long frameCount = 0;

        private int savedCount = 0;
        private final SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US);

        // One-shot capture flag from opmode
        private volatile boolean oneShotCaptureRequested = false;

        // Latest frame for Panels stream
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        public enum SlotState { EMPTY, GREEN, PURPLE }

        // Slot0 triangle points
        private final Point[] tri0Points = new Point[] {
                new Point(335, 400),
                new Point(570, 480),
                new Point(335, 480)
        };
        private final MatOfPoint tri0Mat = new MatOfPoint(tri0Points);

        // Slot1/2 rectangles
        // index: 0 triangle, 1 bottom rect, 2 top rect
        private final Rect[] slotRects = new Rect[] {
                null,
                new Rect(420, 420, 220, 60),  // slot1
                new Rect(330, 70, 230, 70)    // slot2
        };

        private final SlotState[] slotStates = new SlotState[] {
                SlotState.EMPTY, SlotState.EMPTY, SlotState.EMPTY
        };

        public C920Pipeline(Context context, AssetManager assets) {
            this.context = context;
            this.assets = assets;
        }

        public int getSavedCount() { return savedCount; }
        public long getFrameCount() { return frameCount; }
        public SlotState[] getSlotStates() { return slotStates; }

        // Called from opmode when RB is pressed (and capture mode is armed)
        public void requestOneShotCapture() {
            oneShotCaptureRequested = true;
        }

        public void close() {
            try { if (interpreter != null) interpreter.close(); } catch (Exception ignored) {}
            interpreter = null;
        }

        private void ensureTfliteReady() {
            if (interpreter != null) return;
            try {
                MappedByteBuffer model = loadModelFileRobust(assets, context, C920PanelsEOCV.tfliteModelFile);
                interpreter = new Interpreter(model);
            } catch (Exception e) {
                interpreter = null;
            }
        }

        // Robust load: try openFd (fast) else copy to cache and map
        private static MappedByteBuffer loadModelFileRobust(AssetManager am, Context ctx, String assetPath) throws IOException {
            try {
                AssetFileDescriptor afd = am.openFd(assetPath);
                FileInputStream fis = new FileInputStream(afd.getFileDescriptor());
                FileChannel fc = fis.getChannel();
                long startOffset = afd.getStartOffset();
                long declaredLength = afd.getDeclaredLength();
                return fc.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
            } catch (IOException openFdFail) {
                // fallback: copy asset to cache
                File outFile = new File(ctx.getCacheDir(), assetPath);
                if (!outFile.exists() || outFile.length() == 0) {
                    FileOutputStream fos = new FileOutputStream(outFile);
                    byte[] buf = new byte[8192];
                    int n;
                    try (java.io.InputStream is = am.open(assetPath)) {
                        while ((n = is.read(buf)) > 0) fos.write(buf, 0, n);
                    }
                    fos.flush();
                    fos.close();
                }
                FileInputStream fis2 = new FileInputStream(outFile);
                FileChannel fc2 = fis2.getChannel();
                return fc2.map(FileChannel.MapMode.READ_ONLY, 0, outFile.length());
            }
        }

        private void ensureInputBuffer() {
            int in = C920PanelsEOCV.tfliteInputSize;
            int bytesPerChannel = C920PanelsEOCV.tfliteUseFloatInput ? 4 : 1;
            int capacity = 1 * in * in * 3 * bytesPerChannel;
            if (inputBuffer == null || inputBuffer.capacity() != capacity) {
                inputBuffer = ByteBuffer.allocateDirect(capacity);
                inputBuffer.order(ByteOrder.nativeOrder());
            }
        }

        private Rect clampRect(Rect roi, int cols, int rows) {
            return new Rect(
                    Math.max(0, roi.x),
                    Math.max(0, roi.y),
                    Math.min(roi.width,  Math.max(0, cols - roi.x)),
                    Math.min(roi.height, Math.max(0, rows - roi.y))
            );
        }

        private SlotState classifyRoiWithTflite(Mat rgbaFrame, Rect roi) {
            ensureTfliteReady();
            if (interpreter == null) return SlotState.EMPTY;

            Rect bounded = clampRect(roi, rgbaFrame.cols(), rgbaFrame.rows());
            if (bounded.width <= 0 || bounded.height <= 0) return SlotState.EMPTY;

            ensureInputBuffer();
            inputBuffer.rewind();

            Mat sub = rgbaFrame.submat(bounded);
            Imgproc.cvtColor(sub, roiRgb, Imgproc.COLOR_RGBA2RGB);
            Imgproc.resize(roiRgb, roiResized, new Size(C920PanelsEOCV.tfliteInputSize, C920PanelsEOCV.tfliteInputSize));

            byte[] pixel = new byte[3];
            for (int y = 0; y < roiResized.rows(); y++) {
                for (int x = 0; x < roiResized.cols(); x++) {
                    roiResized.get(y, x, pixel); // RGB
                    int r = pixel[0] & 0xFF;
                    int g = pixel[1] & 0xFF;
                    int b = pixel[2] & 0xFF;

                    if (C920PanelsEOCV.tfliteUseFloatInput) {
                        inputBuffer.putFloat(r / 255f);
                        inputBuffer.putFloat(g / 255f);
                        inputBuffer.putFloat(b / 255f);
                    } else {
                        inputBuffer.put((byte) r);
                        inputBuffer.put((byte) g);
                        inputBuffer.put((byte) b);
                    }
                }
            }

            output[0][0] = output[0][1] = output[0][2] = 0f;
            interpreter.run(inputBuffer, output);

            float pEmpty = output[0][0];
            float pGreen = output[0][1];
            float pPurple = output[0][2];

            int best = 0;
            float bestP = pEmpty;
            if (pGreen > bestP) { bestP = pGreen; best = 1; }
            if (pPurple > bestP) { bestP = pPurple; best = 2; }

            if (best == 1 && bestP >= C920PanelsEOCV.tfliteMinConfidence) return SlotState.GREEN;
            if (best == 2 && bestP >= C920PanelsEOCV.tfliteMinConfidence) return SlotState.PURPLE;
            return SlotState.EMPTY;
        }

        private File ensureDir(String labelName) {
            File base = new File("/sdcard/FIRST/roi_dataset");
            File dir = new File(base, labelName.toLowerCase(Locale.US));
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            return dir;
        }

        private void saveRoiPng(Mat rgbaFrame, Rect roi, String prefix, String labelFolder) {
            try {
                Rect bounded = clampRect(roi, rgbaFrame.cols(), rgbaFrame.rows());
                if (bounded.width <= 0 || bounded.height <= 0) return;

                File dir = ensureDir(labelFolder);

                Mat sub = rgbaFrame.submat(bounded);
                Imgproc.cvtColor(sub, roiRgb, Imgproc.COLOR_RGBA2RGB);
                Imgproc.resize(roiRgb, roiResized, new Size(C920PanelsEOCV.captureImageSize, C920PanelsEOCV.captureImageSize));

                Bitmap bmp = Bitmap.createBitmap(roiResized.cols(), roiResized.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(roiResized, bmp);

                String name = prefix + "_" + sdf.format(new Date()) + ".png";
                File out = new File(dir, name);
                FileOutputStream fos = new FileOutputStream(out);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, fos);
                fos.flush();
                fos.close();

                savedCount++;
                sub.release();
            } catch (Exception ignored) { }
        }

        private SlotState hsvClassifySubmat(Mat hsvSub) {
            Scalar greenLower = new Scalar(hsvGreenLowerH, hsvGreenLowerS, hsvGreenLowerV);
            Scalar greenUpper = new Scalar(hsvGreenUpperH, hsvGreenUpperS, hsvGreenUpperV);
            Scalar purpleLower = new Scalar(hsvPurpleLowerH, hsvPurpleLowerS, hsvPurpleLowerV);
            Scalar purpleUpper = new Scalar(hsvPurpleUpperH, hsvPurpleUpperS, hsvPurpleUpperV);

            Core.inRange(hsvSub, greenLower, greenUpper, maskGreen);
            int greenCount = Core.countNonZero(maskGreen);

            Core.inRange(hsvSub, purpleLower, purpleUpper, maskPurple);
            int purpleCount = Core.countNonZero(maskPurple);

            if (greenCount > purpleCount && greenCount > minPixelsForBall) return SlotState.GREEN;
            if (purpleCount > greenCount && purpleCount > minPixelsForBall) return SlotState.PURPLE;
            return SlotState.EMPTY;
        }

        private boolean manualOverrideEnabled(int slotIdx) {
            if (slotIdx == 0) return manualOverrideSlot0;
            if (slotIdx == 1) return manualOverrideSlot1;
            return manualOverrideSlot2;
        }

        private int manualLabelFor(int slotIdx) {
            if (slotIdx == 0) return manualLabelSlot0;
            if (slotIdx == 1) return manualLabelSlot1;
            return manualLabelSlot2;
        }

        private String labelFolderForSlot(int slotIdx, SlotState hsvState) {
            // auto-label from HSV by default
            if (autoLabelFromHSV && !manualOverrideEnabled(slotIdx)) {
                return hsvState.name(); // EMPTY/GREEN/PURPLE
            }
            // manual override wins
            int m = manualLabelFor(slotIdx);
            return labelNameFromId(m);
        }

        @Override
        public Mat processFrame(Mat input) {
            frameCount++;

            if (!masksInitialized) {
                slotMask.create(input.rows(), input.cols(), CvType.CV_8UC1);
                masksInitialized = true;
            }

            // RGBA -> HSV
            Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

            // Debug slider mask (optional)
            Scalar lower = new Scalar(lowerH, lowerS, lowerV);
            Scalar upper = new Scalar(upperH, upperS, upperV);
            Core.inRange(hsv, lower, upper, maskDebug);

            boolean doInfer = useTfliteClassifier
                    && (inferEveryNFrames <= 1 || (frameCount % inferEveryNFrames) == 0);

            // MANUAL capture: only true if opmode requested and capture mode is armed
            boolean doCapture = datasetCaptureEnabled && oneShotCaptureRequested;

            // ===================== Slot 0 (triangle) =====================
            slotMask.setTo(new Scalar(0));
            Imgproc.fillConvexPoly(slotMask, tri0Mat, new Scalar(255));

            // HSV classify triangle by masking counts
            SlotState hsvState0;
            {
                Scalar greenLower = new Scalar(hsvGreenLowerH, hsvGreenLowerS, hsvGreenLowerV);
                Scalar greenUpper = new Scalar(hsvGreenUpperH, hsvGreenUpperS, hsvGreenUpperV);
                Scalar purpleLower = new Scalar(hsvPurpleLowerH, hsvPurpleLowerS, hsvPurpleLowerV);
                Scalar purpleUpper = new Scalar(hsvPurpleUpperH, hsvPurpleUpperS, hsvPurpleUpperV);

                Core.inRange(hsv, greenLower, greenUpper, maskGreen);
                Core.inRange(hsv, purpleLower, purpleUpper, maskPurple);

                Core.bitwise_and(maskGreen, slotMask, slotGreen);
                Core.bitwise_and(maskPurple, slotMask, slotPurple);

                int greenCount0 = Core.countNonZero(slotGreen);
                int purpleCount0 = Core.countNonZero(slotPurple);

                if (greenCount0 > purpleCount0 && greenCount0 > minPixelsForBall) hsvState0 = SlotState.GREEN;
                else if (purpleCount0 > greenCount0 && purpleCount0 > minPixelsForBall) hsvState0 = SlotState.PURPLE;
                else hsvState0 = SlotState.EMPTY;
            }

            Rect triBounds = Imgproc.boundingRect(tri0Mat);

            SlotState finalState0;
            if (useTfliteClassifier) {
                finalState0 = doInfer ? classifyRoiWithTflite(input, triBounds) : slotStates[0];
            } else {
                finalState0 = hsvState0;
            }
            slotStates[0] = finalState0;

            if (doCapture) {
                String folder = labelFolderForSlot(0, hsvState0);
                saveRoiPng(input, triBounds, "slot0", folder);
            }

            // Draw triangle outline colored by FINAL state (TFLite if enabled else HSV)
            Scalar triColor = (finalState0 == SlotState.GREEN) ? new Scalar(0,255,0) :
                    (finalState0 == SlotState.PURPLE) ? new Scalar(255,0,255) :
                            new Scalar(255,255,255);
            Imgproc.polylines(input, Collections.singletonList(tri0Mat), true, triColor, 2);

            // ===================== Slot 1 & 2 (rectangles) =====================
            for (int i = 1; i < slotRects.length; i++) {
                Rect r = slotRects[i];
                if (r == null) continue;

                Rect bounded = clampRect(r, hsv.cols(), hsv.rows());
                if (bounded.width <= 0 || bounded.height <= 0) {
                    slotStates[i] = SlotState.EMPTY;
                    continue;
                }

                // HSV classify ROI
                SlotState hsvState;
                Mat slotHSV = hsv.submat(bounded);
                hsvState = hsvClassifySubmat(slotHSV);
                slotHSV.release();

                SlotState finalState;
                if (useTfliteClassifier) {
                    finalState = doInfer ? classifyRoiWithTflite(input, bounded) : slotStates[i];
                } else {
                    finalState = hsvState;
                }
                slotStates[i] = finalState;

                if (doCapture) {
                    String folder = labelFolderForSlot(i, hsvState);
                    saveRoiPng(input, bounded, "slot" + i, folder);
                }

                Scalar boxColor = (finalState == SlotState.GREEN) ? new Scalar(0,255,0) :
                        (finalState == SlotState.PURPLE) ? new Scalar(255,0,255) :
                                new Scalar(255,255,255);
                Imgproc.rectangle(input, bounded, boxColor, 2);
            }

            // reset one-shot capture flag after we've processed all slots
            if (doCapture) {
                oneShotCaptureRequested = false;
            }

            // Panels stream bitmap
            try {
                if (!input.empty()) {
                    Bitmap b = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
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
            // overlays are drawn in processFrame
        }
    }
}
