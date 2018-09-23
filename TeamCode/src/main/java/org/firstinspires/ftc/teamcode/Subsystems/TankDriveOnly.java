package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Robot with only Tank Drive
 */

public class TankDriveOnly extends Robot {
    public TankDrive drive;

    public TankDriveOnly(OpMode opMode) {
        super(opMode);
    }

    @Override
    protected void addSubsystems(OpMode opMode) {
        try {
            drive = new TankDrive(opMode.hardwareMap);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping TankDrive");
        }
    }
}
