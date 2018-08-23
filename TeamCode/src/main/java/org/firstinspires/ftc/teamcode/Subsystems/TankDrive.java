package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.drive.Drive;
import com.paragonftc.ftc.hardware.CachingDcMotorEx;
import com.paragonftc.ftc.hardware.TankDriveUtil;
import com.paragonftc.ftc.util.TelemetryUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

/**
 * Tank Drive Subsystem
 */

public class TankDrive extends Subsystem {

    public enum Mode {
        OPEN_LOOP,
    }

    public static final String[] LEFT_MOTOR_NAMES = {"tankLeft"};
    public static final String[] RIGHT_MOTOR_NAMES = {"tankRight"};
    public static final int LEFT_MOTOR_NUMBER = LEFT_MOTOR_NAMES.length;
    public static final int RIGHT_MOTOR_NUMBER = RIGHT_MOTOR_NAMES.length;

    public static final double RADIUS = 1.9685;
    public static final double TRACK_WIDTH = 1;
    public static final double GEAR_RATIO = 2;

    private DcMotorEx[] leftMotors;
    private DcMotorEx[] rightMotors;
    public double ticksPerRev = leftMotors[0].getMotorType().getTicksPerRev();

    private int[] driveEncoderOffsets;
    private boolean useCachedDriveEncoderPositions;
    private int[] cachedDriveEncoderPositions;

    private BNO055IMU imu;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private double targetOmega = 0;
    private double targetX = 0;

    private LynxModule Hub1;

    public TankDriveUtil tankDriveUtil = new TankDriveUtil(TRACK_WIDTH, leftMotors, rightMotors, RADIUS, ticksPerRev, GEAR_RATIO);

    private TelemetryData telemetryData;

    public class TelemetryData {
        public Mode driveMode;

        public double leftPower;
        public double rightPower;

        public double leftVel;
        public double rightVel;
    }

    public TankDrive(HardwareMap map) {
        this.telemetryData = new TelemetryData();

        Hub1 = map.get(LynxModule.class, "Hub1");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        powers = new double[2];
        driveEncoderOffsets = new int[4];
        leftMotors = new DcMotorEx[LEFT_MOTOR_NUMBER];
        for (int i = 0; i < LEFT_MOTOR_NUMBER; i ++) {
            DcMotorEx dcMotorEx = map.get(DcMotorEx.class, LEFT_MOTOR_NAMES[i]);
            leftMotors[i] = new CachingDcMotorEx(dcMotorEx);
            leftMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
        }
        rightMotors = new DcMotorEx[RIGHT_MOTOR_NUMBER];
        for (int i = 0; i < RIGHT_MOTOR_NUMBER; i ++) {
            DcMotorEx dcMotorEx = map.get(DcMotorEx.class, LEFT_MOTOR_NAMES[i]);
            rightMotors[i] = new CachingDcMotorEx(dcMotorEx);
            rightMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
        }

        resetDriveEncoders();
    }

    public DcMotor[] getLeftMotors() {
        return leftMotors;
    }
    public DcMotor[] getRightMotors() {
        return rightMotors;
    }

    private void setMode(Mode mode) {
        this.mode = mode;
    }

    private Mode getMode() {
        return mode;
    }

    public void setVelocity(double x, double omega) {
        internalSetVelocity(x, omega);
        mode = Mode.OPEN_LOOP;
    }

    private void internalSetVelocity(double x, double omega) {
        this.targetX = x;
        this.targetOmega = omega;
    }

    private void updatePowers() {
        powers[0] = targetX + targetOmega;
        powers[1] = targetX - targetOmega;

        double max = Collections.max(Arrays.asList(1.0,Math.abs(powers[0]),Math.abs(powers[1])));

        for (int i = 0; i < 2; i ++) {
            powers[i] /= max;
        }
    }

    public void stop() {
        setVelocity(0,0);
    }
    private void resetDriveEncoders() {
        int[] positions = internalGetDriveEncoderPositions();
        for (int i = 0; i < 2; i ++){
            driveEncoderOffsets[i] = -positions[i];
        }
    }

    public double[] getDriveMotorRotations() {
        double[] motorRotations = new double[4];
        int[] encoderPositions = getDriveEncoderPositions();
        for (int i = 0; i < 4; i ++) {
            motorRotations[i] = driveEncoderTicksToRadians(encoderPositions[i]);
        }
        return motorRotations;
    }

    public int[] getDriveEncoderPositions() {
        if (!useCachedDriveEncoderPositions || cachedDriveEncoderPositions == null) {
            cachedDriveEncoderPositions = internalGetDriveEncoderPositions();
            for (int i = 0; i < 4; i ++) {
                cachedDriveEncoderPositions[i] += driveEncoderOffsets[i];
            }
            useCachedDriveEncoderPositions = true;
        }
        return  cachedDriveEncoderPositions;
    }

    private int[] internalGetDriveEncoderPositions() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(Hub1);
        int[] positions = new int[4];
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            for (int i = 0; i < 2; i ++){
                positions[i] = response.getEncoder(i);
            }
            positions[1] = -positions[1];

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            Log.w("TankDrive", e);
        }
        return positions;
    }

    private double driveEncoderTicksToRadians(int ticks) {
        return 2 * ticks / ticksPerRev;
    }

    private void invalidateCaches() {
        useCachedDriveEncoderPositions = false;
    }

    public void setVelocityPIDCoefficients(PIDCoefficients pidCoefficients) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        }
    }

    public Map<String,Object> update(Canvas fieldOverlay) {
        invalidateCaches();

        telemetryData.driveMode = mode;

        updatePowers();

        tankDriveUtil.setMotorPowers(powers[0],powers[1]);

        telemetryData.leftPower = powers[0];
        telemetryData.rightPower = powers[1];
        telemetryData.leftPower = leftMotors[0].getVelocity(AngleUnit.RADIANS);
        telemetryData.rightPower = leftMotors[0].getVelocity(AngleUnit.RADIANS);

        return TelemetryUtil.objectToMap(telemetryData);
    }
}