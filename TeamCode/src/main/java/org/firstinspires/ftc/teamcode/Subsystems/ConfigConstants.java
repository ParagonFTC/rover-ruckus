package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class ConfigConstants {
    public static PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20 , 8, 12);
    public static PIDCoefficients SLOW_VELOCITY_PID = new PIDCoefficients(10 , 3, 1);
}
