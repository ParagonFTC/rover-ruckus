package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;

@TeleOp(name = "TeleOp3 :)", group = "teleop")
public class TeleOp3 extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private FirstRobot robot;

    private boolean slowMode;

    @Override
    public void init() {
        robot = new FirstRobot(this);
        robot.start();
        robot.depot.raise();
        robot.arm.setInitPosition(ScoringArm.CRATER);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();
        telemetry.update();

        //drive
        if (stickyGamepad1.a) {
            slowMode = !slowMode;
        }

        double x, y = 0, omega;

        x = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_x) > 0.5) {
            y = -gamepad1.left_stick_x;
        }

        omega = -gamepad1.right_stick_x;

        if (gamepad2.right_stick_button) {
            omega = -gamepad2.right_stick_x * 0.2;
        }

        if (slowMode) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        robot.drive.setVelocity(new Pose2d(x, y, omega));

        if (gamepad1.dpad_up) {
            robot.latch.setWinchPower(1);
        } else if (gamepad1.dpad_down) {
            robot.latch.setWinchPower(-1);
        } else {
            robot.latch.setWinchPower(0);
        }

        if (gamepad1.x && gamepad1.y) {
            robot.latch.lock();
        } else if (gamepad1.x && !gamepad1.y) {
            robot.latch.unlock();
        }
        robot.arm.setIntakePower(-0.8 * gamepad2.left_stick_y);
        if (!gamepad2.right_stick_button) {
            robot.arm.setJointPower(gamepad2.right_stick_y);
        }

        if (gamepad2.left_trigger > 0.01) {
            robot.arm.setExtensionPower(1);
        } else if (gamepad2.left_bumper) {
            robot.arm.setExtensionPower(-1);
        } else {
            robot.arm.setExtensionPower(0);
        }

        if (gamepad2.right_bumper) {
            robot.arm.enableHold();
        } else if ((gamepad2.right_trigger > 0.01) || (Math.abs(gamepad2.left_stick_y) > 0.01)) {
            robot.arm.disableHold();
        }
    }
}
