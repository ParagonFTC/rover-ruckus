package org.firstinspires.ftc.teamcode.opmodes;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;

/**
 * Standard autonomous op mode with common functions
 */
public abstract class AutoOpMode extends LinearOpMode {
    public static final double SAMPLING_READ_TIMEOUT = 5; // seconds

    protected FirstRobot robot;

    protected SamplingOrderDetector detector;

    protected abstract void setup();
    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new FirstRobot(this);
        robot.start();
/*
        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001;
        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
*/
        robot.latch.lock();

        setup();

        waitForStart();

        run();
    }

    protected void land() {
        robot.latch.setWinchPower(1);
        robot.sleep(0.2);
        robot.latch.unlock();
        robot.latch.setWinchPower(-0.5);
        robot.sleep(1.5);
        robot.latch.setWinchPower(0);
    }
}
