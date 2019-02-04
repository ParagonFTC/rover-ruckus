package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachineScoringArm;

@TeleOp
public class ScoringArmTest extends OpMode {
    private StateMachineScoringArm arm;

    @Override
    public void init() {
        arm = new StateMachineScoringArm(hardwareMap);
    }

    @Override
    public void loop() {
        arm.update();
        arm.setIntakePower(-gamepad2.left_stick_y);
        arm.setJointPower(gamepad2.right_stick_y);
        if (gamepad2.dpad_up) {
            arm.setExtensionPower(1);
        } else if (gamepad2.dpad_down) {
            arm.setExtensionPower(-1);
        } else {
            arm.setExtensionPower(0);
        }
        if (gamepad2.y) {
            arm.raiseArm();
        } else if (gamepad2.a) {
            arm.lowerArm();
        }

        if (gamepad2.x) {
            arm.setReferencePosition();
        }
    }
}
