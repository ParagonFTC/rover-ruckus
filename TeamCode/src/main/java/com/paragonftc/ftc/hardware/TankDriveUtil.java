package com.paragonftc.ftc.hardware;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;
public class TankDriveUtil extends TankDrive {
    private DcMotorEx[] leftMotors;
    private DcMotorEx[] rightMotors;
    private double radius;
    private double ticksPerRev;
    private double gearRatio;
    public TankDriveUtil(double trackWidth, DcMotorEx[] leftMotors, DcMotorEx[] rightMotors, double radius, double ticksPerRev, double gearRatio) {
        super(trackWidth);
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.radius = radius;
        this.ticksPerRev = ticksPerRev;
        this.gearRatio = gearRatio;
    }

    @Override
    public void setMotorPowers(double left, double right) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(left);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(right);
        }
    }

    private double encoderTicksToInches(int ticks) {
        return radius * 2 * Math.PI * gearRatio * ticks / ticksPerRev;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        double lSum = 0.0;
        double rSum = 0.0;
        for (DcMotorEx leftMotor : leftMotors) {
            lSum += encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rSum += encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        wheelPositions.add(lSum/leftMotors.length);
        wheelPositions.add(rSum/rightMotors.length);
        return wheelPositions;
    }
}
