package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;

/**
 * Standard autonomous op mode with common functions
 */
public abstract class AutoOpMode extends LinearOpMode {
    protected NanoClock clock;

    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;

    protected FirstRobot robot;

    protected GoldAlignDetector detector;

    protected abstract void setup();
    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new FirstRobot(this);
        robot.start();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.alignSize = 200;

        clock = NanoClock.system();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        setup();

        waitForStart();
        robot.drive.setHeading(0);
        run();
    }

    protected void land() {
        robot.latch.setWinchPower(1);
        robot.sleep(0.08);
        robot.latch.unlock();
        robot.sleep(0.2);
        robot.latch.setWinchPower(-0.3);
        robot.sleep(1.15);
        robot.latch.setWinchPower(0);
        robot.drive.enableHeadingCorrection(Math.PI / 2);
        while (robot.drive.isMaintainHeading() && opModeIsActive()) {
            idle();
        }
    }

    protected void sample() {
        detector.enable();
        robot.sleep(1);
        robot.drive.setVelocity(new Pose2d(0,0.3,0));
        double startTimeStamp = clock.seconds();
        while (!detector.getAligned() && !isStopRequested()) {
            idle();
        }
        robot.drive.stop();
        double endTimeStamp = clock.seconds();
        double deltaT = endTimeStamp - startTimeStamp;
        detector.disable();
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(1);
        robot.drive.stop();
        robot.drive.setVelocity(new Pose2d(-0.5,0,0));
        robot.sleep(1);
        robot.drive.stop();
        dashboardTelemetry.addData("Delta T", deltaT);
        dashboardTelemetry.update();
        robot.drive.setVelocity(new Pose2d(0,0.3,0));
        robot.sleep(7 - deltaT);
        robot.drive.stop();
    }

    protected void driveToSample() {
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(1.15);
        robot.drive.setVelocity(new Pose2d(0,-0.5,0));
        robot.sleep(1.45);
        robot.drive.stop();
    }

    protected void park() {
        robot.drive.setVelocity(new Pose2d(-1,0,0));
        robot.sleep(4);
        robot.drive.stop();
    }

    protected void claim() {
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(3.5);
        robot.drive.stop();
        robot.depot.lower();
        robot.sleep(0.5);
    }
}
