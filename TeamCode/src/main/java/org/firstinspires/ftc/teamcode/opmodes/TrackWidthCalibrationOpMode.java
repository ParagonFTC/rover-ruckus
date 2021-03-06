package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;

/*
 * This routine measures the effective track width of the drivetrain (i.e., the distance between a
 * pair of wheels on opposite sides of the robot). This is required for the robot turn properly
 * during open-loop control.
 */
@Disabled
@Config
@Autonomous
public class TrackWidthCalibrationOpMode extends LinearOpMode {
    public static int TOTAL_REVOLUTIONS = 4;
    public static double POWER = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveTrain drive = new MecanumDriveTrain(hardwareMap);

        telemetry.log().add("Press play to begin the track width calibration routine");
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.log().add("Additionally, set the drive's track width to 1");
        telemetry.update();

        waitForStart();

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        int revolutions = 0;
        boolean startedMoving = false;
        double lastHeading = 0;

        drive.setPoseEstimate(new Pose2d());
        drive.setVelocity(new Pose2d(0.0, 0.0,  POWER));
        while (opModeIsActive() && (!startedMoving || revolutions <= TOTAL_REVOLUTIONS)) {
            double heading = drive.getExternalHeading();
            if (heading >= Math.PI / 2.0) {
                startedMoving = true;
            }
            if (startedMoving && (lastHeading < 0.0 && heading >= 0.0)) {
                revolutions++;
            }
            drive.updatePoseEstimate();
            lastHeading = heading;
            telemetry.addData("heading", heading);
            telemetry.update();
        }
        drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));
        double effectiveTrackWidth = drive.getPoseEstimate().getHeading() / (4.0 * Math.PI * TOTAL_REVOLUTIONS);

        telemetry.log().clear();
        telemetry.log().add("Calibration complete");
        telemetry.log().add(String.format("effective track width = %.2f", effectiveTrackWidth));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
