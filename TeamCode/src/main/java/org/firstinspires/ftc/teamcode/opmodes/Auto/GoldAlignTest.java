package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
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

        detector.enable();
    }

    @Override
    public void loop() {
        telemetry.addData("IsAligned", detector.getAligned());
        telemetry.addData("X Pos", detector.getXPosition());
    }
}
