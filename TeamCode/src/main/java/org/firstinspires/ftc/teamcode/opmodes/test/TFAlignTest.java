package org.firstinspires.ftc.teamcode.opmodes.test;

import com.paragonftc.ftc.vision.TFGoldAlign;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class TFAlignTest extends OpMode {
    private TFGoldAlign detector;
    @Override
    public void init() {
        detector = new TFGoldAlign(hardwareMap);
    }

    @Override
    public void start() {
        detector.enable();
    }

    @Override
    public void loop() {
        telemetry.addData("ALigned", detector.isAligned());
        telemetry.addData("X Position", detector.getGoldXPos());
        telemetry.addData("Found", detector.isFound());
        telemetry.update();
    }

    @Override
    public void stop() {
        detector.disable();
    }
}
