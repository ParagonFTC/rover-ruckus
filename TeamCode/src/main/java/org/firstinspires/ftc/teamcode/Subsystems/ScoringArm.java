package org.firstinspires.ftc.teamcode.Subsystems;

import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringArm extends Subsystem {
    private DcMotor joint, extension, intake;

    private double jointPower, extensionPower, intakePower;

    public ScoringArm(HardwareMap map) {
        joint = new CachingDcMotor(map.dcMotor.get("joint"));
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extension = new CachingDcMotor(map.dcMotor.get("extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new CachingDcMotor(map.dcMotor.get("intake"));
    }

    public void setJointPower(double jointPower) {
        this.jointPower = jointPower;
    }

    public void setExtensionPower(double extensionPower) {
        this.extensionPower = extensionPower;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    @Override
    public void update() {
        joint.setPower(jointPower);
        extension.setPower(extensionPower);
        intake.setPower(intakePower);
    }
}
