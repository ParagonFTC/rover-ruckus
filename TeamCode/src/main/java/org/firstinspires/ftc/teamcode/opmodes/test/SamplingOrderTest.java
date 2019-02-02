package org.firstinspires.ftc.teamcode.opmodes.test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sampling Order", group = "DogeCV")
public class SamplingOrderTest extends OpMode {
    private SamplingOrderDetector detector;
    @Override
    public void init() {
        telemetry.addData("Status", "Sampling Order Test");

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.enable();
    }

    @Override
    public void loop() {
        telemetry.addData("Current Order", detector.getCurrentOrder().toString());
        telemetry.addData("Last Order", detector.getLastOrder().toString());
    }
}
