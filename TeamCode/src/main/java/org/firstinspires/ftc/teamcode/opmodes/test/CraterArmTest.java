package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;

@TeleOp
public class CraterArmTest extends OpMode {
    FirstRobot robot;

    @Override
    public void init() {
        robot = new FirstRobot(this);
        robot.start();
        robot.crater.raise();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.crater.raise();
        } else if (gamepad1.b) {
            robot.crater.lower();
        }
    }
}
