package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LanderLatcher;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveBase;

@TeleOp(name = "TeleOp", group = "teleop")
public class TeleOp1 extends OpMode {
    private StickyGamepad stickyGamepad1;

    private MecanumDriveBase mecanumDriveBase;
    private LanderLatcher landerLatcher;

    private boolean slowMode;
    @Override
    public void init() {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        landerLatcher = new LanderLatcher(hardwareMap);

        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        mecanumDriveBase.update();

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

        if (slowMode) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        mecanumDriveBase.setVelocity(new Pose2d(x, y, omega));

        if (gamepad1.dpad_up) {
            landerLatcher.setWinchPower(1);
        } else if (gamepad1.dpad_down) {
            landerLatcher.setWinchPower(-1);
        } else {
            landerLatcher.setWinchPower(0);
        }
    }
}
