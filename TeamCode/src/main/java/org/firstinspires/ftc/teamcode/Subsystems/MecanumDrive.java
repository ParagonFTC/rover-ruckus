package org.firstinspires.ftc.teamcode.Subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Vector2d;
import com.paragonftc.ftc.hardware.MecanumDriveUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class MecanumDrive extends Subsystem {
    public static final int IMU_PORT = 0;

    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PATH
    }

    public  static final String[] MOTOR_NAMES = {"driveLf", "driveLb", "driveRb", "driveRf"};

    public static final double TRACK_WIDTH = 18;
    public static final double WHEEL_BASE = 18;

    /**
     * K = (wheelbase width + wheelbase height) / 4 (in)
     */
    public static final double K = (TRACK_WIDTH + WHEEL_BASE) / 4;

    /**
     * wheel radius (in)
     */
    public static final double RADIUS = 1.9685;

    private DcMotorEx[] motors;

    /**
     * units in encoder ticks; solely intended for internal use
     */
    private int[] driveEncoderOffsets;
    private boolean useCachedDriveEncoderPositions;
    private int[] cachedDriveEncoderPositions;

    private BNO055IMU imu;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private Vector2d targetVel = new Vector2d(0,0);
    private double targetOmega = 0;

    private LynxModule centerHub, sideHub;

    private TelemetryData telemetryData;

    public class TelemetryData {
        public Mode driveMode;

        public double frontLeftPower;
        public double rearLeftPower;
        public double rearRightPower;
        public double frontRightPower;
    }

    public MecanumDrive(HardwareMap map) {
        this.telemetryData = new TelemetryData();
    }
    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        return null;
    }
}
