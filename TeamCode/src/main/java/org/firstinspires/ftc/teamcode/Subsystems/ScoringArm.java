package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

@Config
public class ScoringArm extends Subsystem {
    private static final double TICKS_PER_REV = 28 * 256;
    private static final double PULLEY_RADIUS = 0.984252;
    private static final MotorConfigurationType EXTENSION_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double EXTENSION_TICKS_PER_REV = EXTENSION_CONFIG.getTicksPerRev();
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
        INIT,
        DUMP,
        MID,
        CRATER,
        INTAKE,
        CUSTOM
    }

    private static final Map<jointPosition, jointPosition> RAISE_EVENT_MAP = new HashMap<>();
    private static final Map<jointPosition, jointPosition> LOWER_EVENT_MAP = new HashMap<>();
    private static final Map<jointPosition, Double> JOINT_POSITION_MAP = new HashMap<>();
    private static final Map<jointPosition, Double> EXTENSION_POSITION_MAP = new HashMap<>();

    static {
        RAISE_EVENT_MAP.put(jointPosition.INIT, jointPosition.INIT);
        RAISE_EVENT_MAP.put(jointPosition.DUMP, jointPosition.INIT);
        RAISE_EVENT_MAP.put(jointPosition.MID, jointPosition.DUMP);
        RAISE_EVENT_MAP.put(jointPosition.CRATER, jointPosition.MID);
        RAISE_EVENT_MAP.put(jointPosition.INTAKE, jointPosition.CRATER);
        LOWER_EVENT_MAP.put(jointPosition.INIT, jointPosition.DUMP);
        LOWER_EVENT_MAP.put(jointPosition.DUMP,jointPosition.MID);
        LOWER_EVENT_MAP.put(jointPosition.MID, jointPosition.CRATER);
        LOWER_EVENT_MAP.put(jointPosition.CRATER, jointPosition.INTAKE);
        LOWER_EVENT_MAP.put(jointPosition.INTAKE, jointPosition.INTAKE);
        JOINT_POSITION_MAP.put(jointPosition.INTAKE, 0.0);
        JOINT_POSITION_MAP.put(jointPosition.CRATER, Math.PI/4);
        JOINT_POSITION_MAP.put(jointPosition.MID, Math.PI/2);
        JOINT_POSITION_MAP.put(jointPosition.DUMP, 3 * Math.PI / 4);
        EXTENSION_POSITION_MAP.put(jointPosition.CRATER, 12.0);
        EXTENSION_POSITION_MAP.put(jointPosition.MID, 0.0);
        EXTENSION_POSITION_MAP.put(jointPosition.DUMP, 30.0);
        EXTENSION_POSITION_MAP.put(jointPosition.INIT, 0.0);
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

    private static int extensionInchestoTicks(double distance) {
        return (int) (distance * EXTENSION_TICKS_PER_REV/(2 * Math.PI) / PULLEY_RADIUS);
    }

    private static double extensionTickstoInches(int ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / EXTENSION_TICKS_PER_REV;
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

    public void raiseArm() {
        setJointPosition(RAISE_EVENT_MAP.get(currentPosition));
    }

    public void lowerArm() {
        setJointPosition(LOWER_EVENT_MAP.get(currentPosition));
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
                targetPosition = radiansToEncoderTicks(JOINT_POSITION_MAP.get(currentPosition) + referencePosition);
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
        if (currentPosition == jointPosition.INTAKE) {
            disableHold();
            extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double currentExtension = extensionTickstoInches(extension.getCurrentPosition());
            if (currentExtension > EXTENSION_POSITION_MAP.get(jointPosition.CRATER)) {
                extension.setPower(extensionPower);
            }
        } else if (currentPosition == jointPosition.DUMP) {
            //what do I do here?????
        } else {
            enableHold();
            extension.setTargetPosition(extensionInchestoTicks(EXTENSION_POSITION_MAP.get(currentPosition)));
            if (extension.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            extension.setPower(1.0);
        }
        intake.setPower(intakePower);
        holder.setPosition(servoPosition);
    }
}
