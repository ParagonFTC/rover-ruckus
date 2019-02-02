package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DepotClaimer;
import org.firstinspires.ftc.teamcode.Subsystems.FirstRobot;

@TeleOp(name = "MarkerArm")
public class MarkerArmTest extends OpMode {
    private FirstRobot robot;

    @Override
    public void init() {
        robot = new FirstRobot(this);
        robot.start();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.depot.raise();
        } else if (gamepad1.b) {
            robot.depot.lower();
        }
    }
}
