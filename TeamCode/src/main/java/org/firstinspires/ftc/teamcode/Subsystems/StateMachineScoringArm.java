package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.StateMachine;

public class StateMachineScoringArm extends Subsystem {
    private static final double JOINT_TICKS_PER_REV = 28 * 256;
    private static final double PULLEY_RADIUS = 0.984252;
    private static final MotorConfigurationType EXTENSION_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double EXTENSION_TICKS_PER_REV = EXTENSION_CONFIG.getTicksPerRev();

    private DcMotor joint, extension, intake;
    private Servo holder;

    public static double LOCK_POSITION = 0.1;
    public static double
    private class ArmStateMachine extends StateMachine {
        public void start() {
            start(
        }
    }
    @Override
    public void update() {

    }
}
