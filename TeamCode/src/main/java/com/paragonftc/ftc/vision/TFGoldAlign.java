package com.paragonftc.ftc.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.ExecutorService;

public class TFGoldAlign {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AWiNNND/////AAABmXs1dnUBs0nRthBOBC9Kyho3qpyfTGG2bd6vCAGPVkkqo1CjZrX4bGMzfqbqVBTTC0FbNe4v409zGxLeT35LOz17xAiE9za3L2h9QYfG/HXweWFwImnHP3nRd/4BoM5Sufel1Qj3l9nXNd05ddZrVY5lgyU04m6vsLjMoredoYrHBkdzr5RnN495Hjg+sdw0dM8/7Gnrd6nGDaXFtEHQS2LveGgpGzLDeXXP0bany+tGjV7BopDAMUSJxpp5IPgkigxQZiLYXuL736Xc/rXrbHyDQzDfVfYludH0XgCHoSNxNXg2qXXjrAUmhvApMPRceGTjMkRNyRiqoVA0EBFpvOkxBVFFAyFyXU1JjnNiFRcd";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private ExecutorService detectorUpdateExecutor;

    private boolean found = false;
    private boolean aligned = false;
    private double goldXPos = 0;

    public double alignPosOffset = 0;
    public double alignSize = 100;

    public TFGoldAlign (HardwareMap hardwareMap) {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        }

        detectorUpdateExecutor = ThreadPool.newSingleThreadExecutor("detector update");
    }

    private Runnable UpdateRunnable = () -> {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    found = false;
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        found = true;
                        goldXPos = recognition.getLeft();
                    }
                    double alignX = recognition.getImageWidth() / 2 + alignPosOffset;
                    double alignXMin = alignX - (alignSize / 2);
                    double alignXMax = alignX - (alignSize / 2);

                    aligned = (goldXPos < alignXMax && goldXPos > alignXMin);
                }
            }
        }
    };

    public void enable() {
        detectorUpdateExecutor.submit(UpdateRunnable);
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void disable() {
        if (detectorUpdateExecutor != null) {
            detectorUpdateExecutor.shutdownNow();
            detectorUpdateExecutor = null;
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public boolean isFound() {
        return found;
    }

    public boolean isAligned() {
        return aligned;
    }

    public double getGoldXPos() {
        return goldXPos;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
    }
}
