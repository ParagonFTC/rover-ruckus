package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;

@Disabled
@Config
@TeleOp(name = "HeadingPID")
public class HeadingPIDTest extends OpMode {
    private FirstRobot robot;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    public static double targetHeading;
    @Override
    public void init() {
        robot = new FirstRobot(this);
        robot.start();
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.drive.enableHeadingCorrection(Math.toRadians(targetHeading));
        }
        dashboardTelemetry.addData("Target Heading", robot.drive.getHeadingSetpoint());
        dashboardTelemetry.addData("Current Heading", robot.drive.getHeading());
        dashboardTelemetry.addData("Heading Error", robot.drive.getHeadingError());
        dashboardTelemetry.update();
    }
}
