package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class ScoringArm extends Subsystem {
    private static final double TICKS_PER_REV = 28 * 256;
    private static final double PULLEY_RADIUS = 0.984252;
    private static final MotorConfigurationType EXTENSION_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double EXTENSION_TICKS_PER_REV = EXTENSION_CONFIG.getTicksPerRev();
    private DcMotor joint, extension, intake;
    private Servo holder;

    public static double LOCK_POSITION = 0.1;
    public static double UNLOCK_POSITION = 0.45;

    public static double JOINT_POWER = 0.7;

    private double jointPower, extensionPower, intakePower;
    private double servoPosition;
    private double referencePosition;
    private int targetPosition;
    private int extensionPosition;
    private boolean active = false;

    public static double JOINT_INIT_POSITION = 3 * Math.PI / 4;
    public static double JOINT_DUMP_POSITION = 3 * Math.PI / 4;
    public static double JOINT_MID_POSITION = Math.PI / 2;
    public static double JOINT_CRATER_POSITION = 0.4;
    public static double JOINT_INTAKE_POSITION = 0;

    public static double EXTENSION_DUMP_POSITION = 30.0;
    public static double EXTENSION_CRATER_POSITION = 12.0;
    public static double EXTENSION_INIT_POSITION = 0.0;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION
    }

    public static final int INIT = 4, DUMP = 3, MID = 2, CRATER = 1, INTAKE = 0, CUSTOM = 5;

    private int currentPosition = INTAKE;
    private int nextPosition = INTAKE;

    class ArmState {
        private double jointPosition, extensionPosition, servoPosition;
        public ArmState (double jointPosition, double extensionPosition, double servoPosition) {
            this.jointPosition = jointPosition;
            this.extensionPosition = extensionPosition;
            this.servoPosition = servoPosition;
        }

        public double getJointPosition() {
            return jointPosition;
        }

        public double getExtensionPosition() {
            return extensionPosition;
        }

        public double getServoPosition() {
            return servoPosition;
        }
    }

    ArmState[] armStates = {
            new ArmState(JOINT_INTAKE_POSITION, EXTENSION_CRATER_POSITION, UNLOCK_POSITION),
            new ArmState(JOINT_CRATER_POSITION, EXTENSION_CRATER_POSITION, LOCK_POSITION),
            new ArmState(JOINT_MID_POSITION, EXTENSION_CRATER_POSITION, LOCK_POSITION),
            new ArmState(JOINT_DUMP_POSITION, EXTENSION_DUMP_POSITION, LOCK_POSITION),
            new ArmState(JOINT_INIT_POSITION, EXTENSION_INIT_POSITION, LOCK_POSITION)
    };

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
        if (currentPosition == INTAKE || currentPosition == DUMP) {
            disableHold();
        }
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

    public void setJointPosition(int position) {
        currentPosition = position;
        targetPosition = radiansToEncoderTicks(referencePosition - armStates[position].getJointPosition());
        extensionPosition = extensionInchestoTicks(armStates[position].getExtensionPosition());
        //TODO: make update() method work
        mode = Mode.RUN_TO_POSITION;
    }

    public void raiseArm() {
        enableHold();
        nextPosition = currentPosition + 1;
        if (nextPosition >= CUSTOM) {
            nextPosition = currentPosition;
        } else {
            setJointPosition(nextPosition);
            currentPosition = nextPosition;
        }
    }

    public void lowerArm() {
        nextPosition = currentPosition - 1;
        if (nextPosition < INTAKE) {
            nextPosition = currentPosition;
        } else {
            setJointPosition(nextPosition);
            currentPosition = nextPosition;
        }
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

    public int getState() {
        return currentPosition;
    }

    @Override
    public void update() {

        if (mode == Mode.RUN_TO_POSITION) {
            if (!active) {
                joint.setTargetPosition(targetPosition);
                extension.setTargetPosition(extensionPosition);
            } else if (!joint.isBusy()) {
                active = false;
                mode = Mode.OPEN_LOOP;
            }
            if (!(joint.getMode() == DcMotor.RunMode.RUN_TO_POSITION)) {
                joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            internalSetJointPower(JOINT_POWER);
            //extensionPower = 0.7;
        } else if (!(joint.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)){
            joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        joint.setPower(jointPower);
        extension.setPower(extensionPower);
        intake.setPower(intakePower);
        holder.setPosition(servoPosition);
    }
}
