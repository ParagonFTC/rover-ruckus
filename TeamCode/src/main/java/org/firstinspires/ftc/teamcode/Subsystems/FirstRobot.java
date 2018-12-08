package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FirstRobot extends Robot {
    public MecanumDriveBase drive;
    public LanderLatcher latch;
    public DepotClaimer depot;
    public CraterArm crater;
    public ScoringArm arm;

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

        try {
            depot = new DepotClaimer(hardwareMap);
            subsystems.add(depot);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, " skipping Marker");
        }

        try {
            crater = new CraterArm(hardwareMap);
            subsystems.add(crater);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, " skipping Crater");
        }

        try {
            arm = new ScoringArm(hardwareMap);
            subsystems.add(arm);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, " skipping Scoring Arm");
        }
    }
}
