package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;

@Autonomous
public class CraterFacingAuto extends AutoOpMode {
    @Override
    protected void setup() {
        robot.latch.lock();
        robot.depot.raise();
        robot.crater.raise();
    }

    @Override
    protected void run() {
        land();
        driveToSample();
        sample();
        robot.drive.enableHeadingCorrection(-3*Math.PI/4);
        while (robot.drive.isMaintainHeading() && opModeIsActive()) {
            idle();
        }
        robot.drive.setVelocity(new Pose2d(0,-0.5,0));
        robot.sleep(1.5);
        robot.drive.stop();
        robot.drive.setHeading(-3*Math.PI/4);
        robot.drive.enableHeadingCorrection();
        robot.drive.setVelocity(new Pose2d(0,0.3,0));
        robot.sleep(0.5);
        robot.drive.stop();
        robot.drive.setVelocity(new Pose2d(0.5,0,0));
        robot.sleep(0.5);
        claim();
        park();
    }
}
