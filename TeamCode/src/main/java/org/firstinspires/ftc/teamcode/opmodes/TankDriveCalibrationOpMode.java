package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.drive.Drive;
import com.paragonftc.ftc.util.TrackWidthCalibrationOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.TankDrive;

@Autonomous
public class TankDriveCalibrationOpMode extends TrackWidthCalibrationOpMode {

    @Override
    protected Drive initdrive() {

    }

    @Override
    protected BNO055IMU initIMU() {
        return null;
    }
}
