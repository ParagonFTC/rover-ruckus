package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.DepotClaimer;
import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;

@Autonomous
public class DepotFacingAuto extends AutoOpMode {
    @Override
    protected void setup() {
        robot.latch.lock();
        robot.depot.lower();
        robot.depot.raise();
    }

    @Override
    protected void run() {
        land();
        driveToSample();
        sample();
        robot.drive.enableHeadingCorrection(Math.PI/4);
        while (robot.drive.isMaintainHeading() && opModeIsActive()) {
            idle();
        }
        robot.drive.setVelocity(new Pose2d(0,0.7,0));
        robot.sleep(1.5);
        robot.drive.stop();
        robot.drive.setHeading(Math.PI/4);
        robot.drive.enableHeadingCorrection();
        robot.drive.setVelocity(new Pose2d(0,-0.3,0));
        robot.sleep(0.5);
        robot.drive.stop();
        claim();
        park();
    }
}
