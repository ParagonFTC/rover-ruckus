package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.paragonftc.ftc.hardware.CachingDcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;

public class MecanumDriveBase extends Subsystem {
    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PATH
    }

    public static final String[] MOTOR_NAMES = {"frontLeft", "rearLeft", "rearRight", "frontRight"};

    public static final double RADIUS = 1.96850394;

    private DcMotorEx[] motors;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private Pose2d targetVel = new Pose2d(0, 0, 0);

    public MecanumDriveBase (HardwareMap map) {
        powers = new double[4];
        motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i ++) {
            DcMotorEx dcMotorEx = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i] = new CachingDcMotorEx(dcMotorEx);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setVelocity (Pose2d vel) {
        this.targetVel = vel;
        mode = Mode.OPEN_LOOP;
    }

    private void updatePowers() {
        powers[0] = targetVel.getX() - targetVel.getY() - targetVel.getHeading();
        powers[1] = targetVel.getX() + targetVel.getY() - targetVel.getHeading();
        powers[2] = targetVel.getX() - targetVel.getY() + targetVel.getHeading();
        powers[3] = targetVel.getX() + targetVel.getY() + targetVel.getHeading();

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
        }
    }

    public void stop() {
        setVelocity(new Pose2d(0, 0, 0));
    }
    @Override
    public void update() {
        updatePowers();

        for (int i = 0; i < 4; i ++) {
            motors[i].setPower(powers[i]);
        }
    }
}
