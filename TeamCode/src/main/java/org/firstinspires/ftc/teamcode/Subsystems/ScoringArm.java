package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ScoringArm extends Subsystem {
    private DcMotor joint, extension, intake;

    public static int UP_POSITION = 0;
    public static int DOWN_POSITION = 0;

    public static double JOINT_POWER = 0.0;

    private double jointPower, extensionPower, intakePower;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION
    }

    public enum jointPosition {
        UP,
        DOWN
    }

    private Mode mode = Mode.OPEN_LOOP;

    public ScoringArm(HardwareMap map) {
        joint = new CachingDcMotor(map.dcMotor.get("joint"));
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extension = new CachingDcMotor(map.dcMotor.get("extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new CachingDcMotor(map.dcMotor.get("intake"));
    }

    public void setJointPower(double jointPower) {
        mode = Mode.OPEN_LOOP;
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
