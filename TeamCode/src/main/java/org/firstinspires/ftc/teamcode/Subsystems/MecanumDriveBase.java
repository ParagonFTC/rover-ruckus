package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.paragonftc.ftc.util.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class MecanumDriveBase extends Subsystem {
    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(0,0,0);

    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PATH
    }

    public MecanumDriveTrain mecanumDriveTrain;

    private double headingOffset;
    private double headingErrorAllowance = Math.toRadians(5);
    private PIDController maintainHeadingController;
    private boolean maintainHeading;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private Pose2d targetVel = new Pose2d(0, 0, 0);

    public MecanumDriveBase (HardwareMap map) {
        powers = new double[4];

        mecanumDriveTrain = new MecanumDriveTrain(map);

        maintainHeadingController = new PIDController(MAINTAIN_HEADING_PID);
        maintainHeadingController.setInputBounds(-Math.PI, Math.PI);
    }

    public void enableHeadingCorrection(double desiredHeading) {
        maintainHeading = true;
        this.maintainHeadingController.setSetpoint(desiredHeading);
        this.maintainHeadingController.reset();
    }

    public void disableHeadingCorrection() {
        maintainHeading = false;
    }

    public void setTargetHeading(double heading) {
        this.maintainHeadingController.setSetpoint(heading);
    }

    public void setHeading(double heading) {
        headingOffset = -mecanumDriveTrain.getExternalHeading() + heading;
    }

    public void setVelocity (Pose2d vel) {
        internalSetVelocity(vel);
        mode = Mode.OPEN_LOOP;
    }

    private void internalSetVelocity (Pose2d vel) {
        this.targetVel = vel;
    }

    public double getHeading() {
        return Angle.norm(mecanumDriveTrain.getExternalHeading() + headingOffset);
    }

    private void updatePowers() {
        mecanumDriveTrain.setVelocity(targetVel);
    }

    public void stop() {
        setVelocity(new Pose2d(0, 0, 0));
    }

    @Override
    public void update() {
        if (maintainHeading) {
            double heading = mecanumDriveTrain.getExternalHeading();
            double headingError = maintainHeadingController.getError(heading);
            double headingUpdate = maintainHeadingController.update(headingError);
            internalSetVelocity(new Pose2d(targetVel.getX(), targetVel.getY(), headingUpdate));
            if (Math.abs(headingError) <= headingErrorAllowance) {
                maintainHeading = false;
            }
        } else {
            internalSetVelocity(targetVel);
        }

        updatePowers();
    }
}
