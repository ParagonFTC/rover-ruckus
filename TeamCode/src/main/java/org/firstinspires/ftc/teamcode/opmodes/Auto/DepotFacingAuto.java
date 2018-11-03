package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;

@Autonomous
public class DepotFacingAuto extends AutoOpMode {
    @Override
    protected void setup() {
        robot.latch.lock();
    }

    @Override
    protected void run() {
        land();
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(1.15);
        robot.drive.setVelocity(new Pose2d(0,-0.5,0));
        robot.sleep(1.35);
        robot.drive.stop();
        sample();
    }
}
