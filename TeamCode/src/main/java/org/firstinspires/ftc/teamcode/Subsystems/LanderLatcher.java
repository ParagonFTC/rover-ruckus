package org.firstinspires.ftc.teamcode.Subsystems;

import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LanderLatcher extends Subsystem {
    public static final double LOCK_POSITION = 0.1;
    public static final double UNLOCK_POSITION = 0.6;
    private double lockPosition;
    private DcMotor winchMotor;
    private Servo lock;

    public enum LatchMode {
        PID,
        MANUAL
    }

    private double winchPower;

    private LatchMode latchMode = LatchMode.MANUAL;

    public LanderLatcher (HardwareMap map) {
        winchMotor = new CachingDcMotor(map.dcMotor.get("winchMotor"));
        lock = new CachingServo(map.get(Servo.class, "lock"));
    }

    public void setWinchPower(double power) {
        winchPower = power;
        latchMode = LatchMode.MANUAL;
    }

    public void setLockPosition (double position) {
        lockPosition = position;
    }

    public void lock() {
        setLockPosition(LOCK_POSITION);
    }

    public void unlock() {
        setLockPosition(UNLOCK_POSITION);
    }
    @Override
    public void update() {
        winchMotor.setPower(winchPower);
        lock.setPosition(lockPosition);
    }
}
