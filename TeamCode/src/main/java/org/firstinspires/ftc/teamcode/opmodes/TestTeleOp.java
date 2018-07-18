package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ConfigConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode{
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private Robot robot;

    private Boolean slowMode, superSlowMode;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        // drive
        if (stickyGamepad1.b) {
            slowMode = !slowMode;
            superSlowMode = false;
            if (slowMode) {
                robot.drive.setVelocityPIDCoefficients(ConfigConstants.SLOW_VELOCITY_PID);
            } else {
                robot.drive.setVelocityPIDCoefficients(ConfigConstants.NORMAL_VELOCITY_PID);
            }
        } else if (stickyGamepad1.left_bumper) {
            superSlowMode = !superSlowMode;
            slowMode = false;
            if (superSlowMode) {
                robot.drive.setVelocityPIDCoefficients(ConfigConstants.SLOW_VELOCITY_PID);
            } else {
                robot.drive.setVelocityPIDCoefficients(ConfigConstants.NORMAL_VELOCITY_PID);
            }
        }

        double x = 0, omega;

        x = gamepad1.left_stick_y;

        omega = gamepad1.right_stick_x;

        if (superSlowMode) {
            x *= 0.25;
            omega *= 0.25;
        } else  if (slowMode) {
            x *= 0.5;
            omega *= 0.5;
        }

        robot.drive.setVelocity(x, omega);
    }
}
