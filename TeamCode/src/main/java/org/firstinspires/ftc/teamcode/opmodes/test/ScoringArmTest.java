package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;

public class ScoringArmTest extends OpMode {
    private FirstRobot robot;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    @Override
    public void init() {
        robot = new FirstRobot(this);
        robot.start();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addLine("ready");
    }

    @Override
    public void loop() {
        robot.arm.setIntakePower(gamepad2.left_stick_y);

        if (gamepad2.dpad_up) {
            robot.arm.setExtensionPower(1);
        } else if (gamepad2.dpad_down) {
            robot.arm.setExtensionPower(-1);
        } else {
            robot.arm.setExtensionPower(0);
        }
    }
}
