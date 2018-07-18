package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
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
import org.firstinspires.ftc.teamcode.Utils.TelemetryUtil;

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

    public static final String[] MOTOR_NAMES = {"tankLeft","tankRight"};

    public static final double RADIUS = 1.9685;

    private DcMotorEx[] motors;

    private int[] driveEncoderOffsets;
    private boolean useCachedDriveEncoderPositions;
    private int[] cachedDriveEncoderPositions;

    private BNO055IMU imu;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private double targetOmega = 0;
    private double targetX = 0;

    private LynxModule Hub1;

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
        motors = new DcMotorEx[2];
        for (int i = 0; i<2; i++) {
            DcMotorEx dcMotorEx = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i] = new CachingDcMotorEx(dcMotorEx);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        resetDriveEncoders();
    }

    public DcMotor[] getMotors() {
        return motors;
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
        double ticksPerRev = motors[0].getMotorType().getTicksPerRev();
        return 2 * ticks / ticksPerRev;
    }

    private void invalidateCaches() {
        useCachedDriveEncoderPositions = false;
    }

    public void setVelocityPIDCoefficients(PIDCoefficients pidCoefficients) {
        for (int i = 0; i < 2; i ++) {
            motors[i].setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        }
    }

    public Map<String,Object> update(Canvas fieldOverlay) {
        invalidateCaches();

        telemetryData.driveMode = mode;

        updatePowers();

        for (int i = 0; i < 2; i ++) {
            motors[i].setPower(powers[i]);
        }

        telemetryData.leftPower = powers[0];
        telemetryData.rightPower = powers[1];
        telemetryData.leftPower = motors[0].getVelocity(AngleUnit.RADIANS);
        telemetryData.rightPower = motors[1].getVelocity(AngleUnit.RADIANS);

        return TelemetryUtil.objectToMap(telemetryData);
    }
}
