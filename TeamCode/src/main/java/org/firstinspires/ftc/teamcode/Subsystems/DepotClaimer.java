package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DepotClaimer extends Subsystem {
    public static double ARM_UP_POSITION = 0.3;
    public static double ARM_DOWN_POSITION = 0.9;

    private double armPosition;
    private Servo markerArm;

    public DepotClaimer (HardwareMap map) {
        markerArm = new CachingServo(map.servo.get("markerArm"));
    }

    public void raise() {
        setArmPosition(ARM_UP_POSITION);
    }

    public void lower() {
        setArmPosition(ARM_DOWN_POSITION);
    }

    public void setArmPosition(double position) {
        armPosition = position;
    }

    @Override
    public void update() {
        markerArm.setPosition(armPosition);
    }
}
