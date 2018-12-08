package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CraterArm extends Subsystem {
    public static double ARM_UP_POSITION = 0.3;
    public static double ARM_DOWN_POSITION = 0.6;

    private double armPosition;
    private Servo craterArm;

    public CraterArm (HardwareMap map) {
        craterArm = new CachingServo(map.servo.get("craterArm"));
    }

    public void raise() {
        setArmPosition(ARM_UP_POSITION);
    }

    public void lower() {
        setArmPosition(ARM_DOWN_POSITION);
    }

    private void setArmPosition(double position) {
        armPosition = position;
    }
    @Override
    public void update() {
        craterArm.setPosition(armPosition);
    }
}
