package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.paragonftc.ftc.hardware.CachingDcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class MecanumDriveBase extends Subsystem {
    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PATH
    }

    public MecanumDriveTrain mecanumDriveTrain;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private Pose2d targetVel = new Pose2d(0, 0, 0);

    public MecanumDriveBase (HardwareMap map) {
        powers = new double[4];

        mecanumDriveTrain = new MecanumDriveTrain(map);
    }

    public void setVelocity (Pose2d vel) {
        this.targetVel = vel;
        mode = Mode.OPEN_LOOP;
    }

    private void updatePowers() {
        mecanumDriveTrain.setVelocity(targetVel);
    }

    public void stop() {
        setVelocity(new Pose2d(0, 0, 0));
    }

    @Override
    public void update() {
        updatePowers();
        mecanumDriveTrain.update();
    }
}
