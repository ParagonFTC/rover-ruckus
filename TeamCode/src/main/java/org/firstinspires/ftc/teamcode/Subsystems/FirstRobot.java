package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FirstRobot extends Robot {
    public MecanumDriveBase drive;
    public LanderLatcher latch;
    public FirstRobot(OpMode opMode) {
        super(opMode);
    }

    @Override
    protected void initHardware(HardwareMap hardwareMap) {
        try {
            drive = new MecanumDriveBase(hardwareMap);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping MecanumDrive");
        }

        try {
            latch = new LanderLatcher(hardwareMap);
            subsystems.add(latch);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, " skipping Latcher");
        }
    }
}
