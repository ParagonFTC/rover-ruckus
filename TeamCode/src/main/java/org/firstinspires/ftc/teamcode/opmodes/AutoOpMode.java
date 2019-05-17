package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.paragonftc.ftc.util.AutoTransitioner;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;

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
        robot.arm.setInitPosition(ScoringArm.INIT);
        robot.arm.enableHold();
        robot.arm.resetExtenison();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        clock = NanoClock.system();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        setup();

        AutoTransitioner.transitionOnStop(this, "TeleOp2");
        waitForStart();
        robot.drive.setHeading(0);
        run();
        while (!isStopRequested()) {
            idle();
        }
    }

    protected void land() {
        robot.latch.setWinchPower(-1);
        robot.arm.setCustomJointPosition(3 * Math.PI / 4);
        robot.arm.setMode(ScoringArm.Mode.RUN_TO_POSITION);
        robot.sleep(0.08);
        robot.arm.setExtensionPower(0);
        robot.latch.unlock();
        robot.sleep(0.2);
        robot.latch.setWinchPower(0.3);
        robot.sleep(1.15);
        robot.latch.setWinchPower(0);
        robot.drive.enableHeadingCorrection(Math.PI / 2);
        while (robot.drive.isMaintainHeading() && opModeIsActive()) {
            idle();
        }
        robot.drive.disableHeadingCorrection();
    }

    protected void sample() {
        detector.enable();
        robot.sleep(1);
        robot.drive.setVelocity(new Pose2d(-0.02,0.3,0));
        double startTimeStamp = clock.seconds();
        while (!detector.getAligned() && !isStopRequested() && clock.seconds() - startTimeStamp < 7) {
            telemetry.addData("Gold X position", detector.getXPosition());
            telemetry.addData("Found", detector.isFound());
            telemetry.addData("Is Aligned", detector.getAligned());
            telemetry.update();
        }
        telemetry.addData("Gold X position", detector.getXPosition());
        telemetry.addData("Found", detector.isFound());
        telemetry.addData("Is Aligned", detector.getAligned());
        telemetry.update();
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
        robot.drive.setVelocity(new Pose2d(-0.02,0.3,0));
        robot.sleep(7 - deltaT);
        robot.drive.stop();
    }

    protected void driveToSample() {
        robot.latch.setWinchPower(-1);
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(1.15);
        robot.latch.setWinchPower(0);
        robot.drive.setVelocity(new Pose2d(0,-0.5,0));
        robot.sleep(1.45);
        robot.drive.stop();
    }

    protected void park() {
        robot.drive.disableHeadingCorrection();
        robot.drive.setVelocity(new Pose2d(-1,0,0));
        robot.sleep(2.3);
        robot.drive.stop();
        robot.arm.setJointPosition(ScoringArm.CRATER);
    }

    protected void claim() {
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(3);
        robot.drive.stop();
        robot.depot.lower();
        robot.sleep(0.5);
    }
}
