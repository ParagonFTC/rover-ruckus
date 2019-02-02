package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gold Align Test")
public class GoldAlignTest extends OpMode {
    private GoldAlignDetector detector;

    @Override
    public void init() {
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.alignSize = 60;

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.perfectAreaScorer.perfectArea = 5000;
        detector.perfectAreaScorer.weight = 1.0;
        detector.ratioScorer.weight = 0;

        detector.enable();
    }

    @Override
    public void loop() {
        telemetry.addData("IsAligned", detector.getAligned());
        telemetry.addData("X Pos", detector.getXPosition());
    }

    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }
}
