package com.paragonftc.ftc.hardware;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

public class MecanumDriveUtil extends MecanumDrive {
    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;
    private double radius;
    private double ticksPerRev;
    private double gearRatio;
    public MecanumDriveUtil(double trackWidth, double wheelBase, DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb, double radius, double ticksPerRev, double gearRatio) {
        super(trackWidth, wheelBase);
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;
        this.radius = radius;
        this.ticksPerRev = ticksPerRev;
        this.gearRatio = gearRatio;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        lf.setPower(v);
        lb.setPower(v1);
        rb.setPower(v2);
        rf.setPower(v3);
    }

    private double encoderTicksToInches(int ticks) {
        return radius * 2 * Math.PI * gearRatio * ticks / ticksPerRev;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> positions = new LinkedList<Double>();
        positions.add(encoderTicksToInches(lf.getCurrentPosition()));
        positions.add(encoderTicksToInches(lb.getCurrentPosition()));
        positions.add(encoderTicksToInches(rf.getCurrentPosition()));
        positions.add(encoderTicksToInches(rb.getCurrentPosition()));
        return positions;
    }
}
