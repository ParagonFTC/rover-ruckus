package org.firstinspires.ftc.teamcode.Subsystems;

import com.paragonftc.ftc.hardware.CachingDcMotor;
import com.paragonftc.ftc.hardware.CachingServo;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Event;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.StateTransition;

public class StateMachineScoringArm extends Subsystem {
    private static final double JOINT_TICKS_PER_REV = 28 * 256;
    private static final double PULLEY_RADIUS = 0.984252;
    private static final MotorConfigurationType EXTENSION_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double EXTENSION_TICKS_PER_REV = EXTENSION_CONFIG.getTicksPerRev();

    private DcMotor joint, extension, intake;
    private Servo holder;

    private boolean stateMachineEnabled = false;

    public static double LOCK_POSITION = 0.1;
    public static double UNLOCK_POSITION = 0.45;

    private double jointPower, extensionPower, intakePower;
    private double servoPosition;
    private int jointTargetPosition, extensionTargetPosition;
    private double referencePosition;

    public static double JOINT_INIT_POSITION = 3 * Math.PI / 4;
    public static double JOINT_DUMP_POSITION = 3 * Math.PI / 4;
    public static double JOINT_MID_POSITION = Math.PI / 2;
    public static double JOINT_CRATER_POSITION = Math.PI / 3;
    public static double JOINT_INTAKE_POSITION = 0;

    public static double EXTENSION_DUMP_POSITION = 30.0;
    public static double EXTENSION_CRATER_POSITION = 12.0;
    public static double EXTENSION_INIT_POSITION = 0.0;

    public enum JointMode {
        OPEN_LOOP,
        RUN_TO_POSITION
    }

    public enum IntakeMode {
        ENABLED,
        DISABLED
    }

    public enum ExtensionMode {
        OPEN_LOOP,
        RUN_TO_POSITION
    }

    JointMode jointMode = JointMode.OPEN_LOOP;
    IntakeMode intakeMode = IntakeMode.DISABLED;
    ExtensionMode extensionMode = ExtensionMode.RUN_TO_POSITION;


    private static final Event RAISE = new Event() {
        @Override
        public String getName() {
            return "raise";
        }
    };

    private static final Event LOWER = new Event() {
        @Override
        public String getName() {
            return "lower";
        }
    };
/*
    private State INIT = new State() {
        @Override
        public void onEnter(Event event) {
            setJointPosition(JOINT_INIT_POSITION);
            internalSetExtentionPosition(EXTENSION_INIT_POSITION);
            servoPosition = LOCK_POSITION;
        }

        @Override
        public void onExit(Event event) {

        }
    };

    private State DUMP = new State() {
        @Override
        public void onEnter(Event event) {
            setJointPosition(JOINT_DUMP_POSITION);
            internalSetExtentionPosition(EXTENSION_DUMP_POSITION);
            intakeMode = IntakeMode.ENABLED;
        }

        @Override
        public void onExit(Event event) {
            intakeMode = IntakeMode.DISABLED;
        }
    };
*/
    private ArmState dump = new ArmState(JOINT_DUMP_POSITION, EXTENSION_DUMP_POSITION, LOCK_POSITION);
    private ArmState mid = new ArmState(JOINT_MID_POSITION, EXTENSION_CRATER_POSITION, LOCK_POSITION);
    private ArmState crater = new ArmState(JOINT_CRATER_POSITION, EXTENSION_CRATER_POSITION, LOCK_POSITION);
    private ArmState mineralGrab = new ArmState(JOINT_INTAKE_POSITION, EXTENSION_CRATER_POSITION, UNLOCK_POSITION);
    private ArmState init = new ArmState(JOINT_INIT_POSITION, EXTENSION_INIT_POSITION, LOCK_POSITION);
    /*
    private State MID = new State() {
        @Override
        public void onEnter(Event event) {
            setJointPosition(JOINT_MID_POSITION);
            internalSetExtentionPosition(EXTENSION_CRATER_POSITION);
            servoPosition = LOCK_POSITION;
        }

        @Override
        public void onExit(Event event) {

        }
    };

    private State CRATER = new State() {
        @Override
        public void onEnter(Event event) {
            setJointPosition(JOINT_CRATER_POSITION);
            internalSetExtentionPosition(EXTENSION_CRATER_POSITION);
            servoPosition = LOCK_POSITION;
        }

        @Override
        public void onExit(Event event) {

        }
    };

    private State INTAKE = new State() {
        @Override
        public void onEnter(Event event) {
            extensionMode = ExtensionMode.OPEN_LOOP;
            setJointPosition(JOINT_INTAKE_POSITION);
            servoPosition = UNLOCK_POSITION;
            internalSetIntakePower(-1.0);
        }

        @Override
        public void onExit(Event event) {
            internalSetIntakePower(0);
            extensionMode = ExtensionMode.RUN_TO_POSITION;
        }
    }; */

    class ArmState implements State {
        private double jointPosition;
        private double extensionPosition;
        private double servoPosition;

        public ArmState (double jointPosition, double extensionPosition, double servoPosition) {
            this.jointPosition = jointPosition;
            this.extensionPosition = extensionPosition;
            this.servoPosition = servoPosition;
        }

        @Override
        public void onEnter(Event event) {
            if (this.jointPosition == JOINT_CRATER_POSITION) {
                if (this.servoPosition != LOCK_POSITION) {
                    enableHold();
                }
            }
        }

        @Override
        public void onExit(Event event) {

        }
    }

    private State[] armStates = {INIT, DUMP, MID, CRATER, INTAKE};

    private StateTransition[] raiseTransitions, lowerTransitions;

    private class ArmStateMachine extends StateMachine {
        public void start(State state) {
            super.start(state);
        }
    }

    private ArmStateMachine armStateMachine;

    public StateMachineScoringArm(HardwareMap map) {
        joint = new CachingDcMotor(map.dcMotor.get("joint"));
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extension = new CachingDcMotor(map.dcMotor.get("extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new CachingDcMotor(map.dcMotor.get("intake"));

        holder = new CachingServo(map.servo.get("holder"));
        armStateMachine = new ArmStateMachine();

        raiseTransitions = new StateTransition[armStates.length];
        raiseTransitions[0] = new StateTransition(INIT, RAISE, INIT);
        for (int i = 1; i < armStates.length; i ++) {
            raiseTransitions[i] = new StateTransition(armStates[i], RAISE, armStates[i - 1]);
        }
        for (int i = 0; i < raiseTransitions.length; i ++) {
            armStateMachine.addTransition(raiseTransitions[i]);
        }

        lowerTransitions = new StateTransition[armStates.length];
        for (int i = 0; i < armStates.length - 1; i ++) {
            lowerTransitions[i] = new StateTransition(armStates[i], LOWER, armStates[i + 1]);
        }
        lowerTransitions[armStates.length - 1] = new StateTransition(INTAKE, LOWER, INTAKE);
        for (int i = 0; i < lowerTransitions.length; i ++) {
            armStateMachine.addTransition(lowerTransitions[i]);
        }
    }
    private static double jointTickstoRadians(int ticks) {
        return 2 * Math.PI * 0.5 * ticks / JOINT_TICKS_PER_REV;
    }

    private static int jointRadiansToTicks(double radians) {
        return (int) (JOINT_TICKS_PER_REV *  radians / Math.PI / 2 / 0.5);
    }

    private static double extensionTickstoInches(int ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / EXTENSION_TICKS_PER_REV;
    }

    private static int extensionInchestoTicks(double inches) {
        return (int) (inches * EXTENSION_TICKS_PER_REV /2 / Math.PI / PULLEY_RADIUS);
    }

    private void setJointPosition(double radians) {
        jointMode = JointMode.RUN_TO_POSITION;
        jointTargetPosition = jointRadiansToTicks(referencePosition + radians);
    }

    private void internalSetExtentionPosition(double inches) {
        extensionTargetPosition = extensionInchestoTicks(inches);
    }

    public void setIntakePower(double intakePower) {
        if (intakeMode == IntakeMode.ENABLED) {
            internalSetIntakePower(intakePower);
        }
    }

    public void enableHold() {
        if (intakeMode == IntakeMode.ENABLED) {
            servoPosition = LOCK_POSITION;
        }
    }

    public void disableHold() {
        if (intakeMode == IntakeMode.ENABLED) {
            servoPosition = UNLOCK_POSITION;
        }
    }

    public void raiseArm() {
        if (stateMachineEnabled) {
            armStateMachine.consumeEvent(RAISE);
        }
    }

    public void lowerArm() {
        if (stateMachineEnabled) {
            armStateMachine.consumeEvent(LOWER);
        }
    }

    public double getJointPosition() {
        return jointTickstoRadians(joint.getCurrentPosition());
    }

    public void setReferencePosition() {
        if (!stateMachineEnabled) {
            stateMachineEnabled = true;
            armStateMachine.start(INTAKE);
        } else {
            stateMachineEnabled = false;
        }
        referencePosition = getJointPosition();
    }

    private void internalSetIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void setExtensionPower(double extensionPower) {
        this.extensionPower = extensionPower;
    }

    public void setJointPower(double jointPower) {
        internalSetJointPower(jointPower);
        if (Math.abs(jointPower) > 0.1) {
            jointMode = JointMode.OPEN_LOOP;
            extensionMode = ExtensionMode.OPEN_LOOP;
            intakeMode = IntakeMode.ENABLED;
        }
    }

    private void internalSetJointPower(double jointPower) {
        this.jointPower = jointPower;
    }

    @Override
    public void update() {
        if (jointMode == JointMode.RUN_TO_POSITION) {
            joint.setTargetPosition(jointTargetPosition);
            joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            internalSetJointPower(0.7);
        } else {
            joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (extensionMode == ExtensionMode.RUN_TO_POSITION) {
            extension.setTargetPosition(extensionTargetPosition);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setExtensionPower(1.0);
        } else {
            extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        joint.setPower(jointPower);
        extension.setPower(extensionPower);
        intake.setPower(intakePower);
        holder.setPosition(servoPosition);
    }
}
