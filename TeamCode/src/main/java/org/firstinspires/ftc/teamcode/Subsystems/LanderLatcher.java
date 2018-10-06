package org.firstinspires.ftc.teamcode.Subsystems;

import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LanderLatcher extends Subsystem {
    private DcMotor winchMotor;

    public enum LatchMode {
        PID,
        MANUAL
    }

    private double winchPower;

    private LatchMode latchMode = LatchMode.MANUAL;

    public LanderLatcher (HardwareMap map) {
        winchMotor = new CachingDcMotor(map.dcMotor.get("winchMotor"));
    }

    public void setWinchPower(double power) {
        winchPower = power;
        latchMode = LatchMode.MANUAL;
    }
    @Override
    public void update() {
        winchMotor.setPower(winchPower);
    }
}
