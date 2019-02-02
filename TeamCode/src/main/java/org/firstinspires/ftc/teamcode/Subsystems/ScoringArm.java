package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class ScoringArm extends Subsystem {
    private static final double TICKS_PER_REV = 28 * 256;
    private DcMotor joint, extension, intake;
    private Servo holder;

    public static double LOCK_POSITION = 0.1;
    public static double UNLOCK_POSITION = 0.6;

    public static double JOINT_POWER = 0.7;

    private double jointPower, extensionPower, intakePower;
    private double servoPosition;
    private jointPosition currentPosition;
    private double referencePosition;
    private int targetPosition;
    private boolean active = false;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION
    }

    public enum jointPosition {
        UP,
        DOWN,
        CUSTOM
    }

    private Mode mode = Mode.OPEN_LOOP;

    public ScoringArm(HardwareMap map) {
        joint = new CachingDcMotor(map.dcMotor.get("joint"));
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extension = new CachingDcMotor(map.dcMotor.get("extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new CachingDcMotor(map.dcMotor.get("intake"));

        holder = new CachingServo(map.servo.get("holder"));
    }

    public void setJointPower(double jointPower) {
        if (Math.abs(jointPower) > 0.1) {
            mode = Mode.OPEN_LOOP;
        }
        internalSetJointPower(jointPower);
    }

    private void internalSetJointPower(double jointPower) {
        this.jointPower = jointPower;
    }

    public void enableHold() {
        servoPosition = LOCK_POSITION;
    }

    public void disableHold() {
        servoPosition = UNLOCK_POSITION;
    }
    public void setExtensionPower(double extensionPower) {
        this.extensionPower = extensionPower;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    private static double encoderTickstoRadians(int ticks) {
        return 2 * Math.PI * 0.5 * ticks / TICKS_PER_REV;
    }

    private static int radiansToEncoderTicks(double radians) {
        return (int) (TICKS_PER_REV *  radians / Math.PI / 2 / 0.5);
    }

    public double getArmPosition() {
        return encoderTickstoRadians(joint.getCurrentPosition());
    }

    public void setJointPosition(jointPosition position) {
        currentPosition = position;
        mode = Mode.RUN_TO_POSITION;
    }

    public void setJointPosition(double radians) {
        targetPosition = radiansToEncoderTicks(referencePosition + radians);
        mode = Mode.RUN_TO_POSITION;
        currentPosition = jointPosition.CUSTOM;
    }

    public void setReferencePosition() {
        referencePosition = getArmPosition();
    }

    public double getReferencePosition() {
        return referencePosition;
    }

    public boolean isActive() {
        return active;
    }

    public double getTargetPosition() {
        return encoderTickstoRadians(this.targetPosition);
    }

    public Mode getMode() {
        return mode;
    }

    @Override
    public void update() {
        if (mode == Mode.RUN_TO_POSITION) {
            if (!active) {
                if (currentPosition == jointPosition.UP) {
                    targetPosition = radiansToEncoderTicks(referencePosition - (Math.PI * 2 / 3));
                } else if (currentPosition == jointPosition.DOWN){
                    targetPosition = radiansToEncoderTicks(referencePosition);
                }
                joint.setTargetPosition(targetPosition);
            } else if (!joint.isBusy()) {
                active = false;
                mode = Mode.OPEN_LOOP;
                internalSetJointPower(0);
            }
            if (!(joint.getMode() == DcMotor.RunMode.RUN_TO_POSITION)) {
                joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            internalSetJointPower(JOINT_POWER);
        } else if (!(joint.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)){
            joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        joint.setPower(jointPower);
        extension.setPower(extensionPower);
        intake.setPower(intakePower);
        holder.setPosition(servoPosition);
    }
}
