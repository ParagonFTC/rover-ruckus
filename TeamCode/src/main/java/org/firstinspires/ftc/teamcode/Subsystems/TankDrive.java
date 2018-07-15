package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

/**
 * Tank Drive Subsystem
 */

public class TankDrive extends Subsystem {

    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PATH,
    }

    public static final String[] MOTOR_NAMES = {"tankLeft","tankRight"};

    public static final double RADIUS = 1.9685;

    private DcMotor[] motors;

    private int[] driveEncoderOffsets;
    private boolean useCachedDriveEncoderPositions;
    private int[] cachedDriveEncoderPositions;

    private BNO055IMU imu;

    private Mode mode = Mode.OPEN_LOOP;

    private LynxModule Hub1;

    private TelemetryData telemetryData;

    public class TelemetryData {
        public Mode driveMode;

        public double leftPower;
        public double rightPower;
    }

    public TankDrive(HardwareMap map) {
        this.telemetryData = new TelemetryData();

        Hub1 = map.get(LynxModule.class, "Hub1");

        driveEncoderOffsets = new int[4];
        for (int i = 0; i<2; i++) {
            motors[i] = map.get(DcMotor.class, MOTOR_NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public DcMotor[] getMotors() {
        return motors;
    }

    private void setMode(Mode mode) {
        this.mode = mode;
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

    public Map<String, Object> update(Canvas fieldOverlay) {
        invalidateCaches();

        telemetryData.driveMode = mode;

    }
}
