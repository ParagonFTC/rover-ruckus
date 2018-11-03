package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;

@Autonomous(name = "testLand", group = "test")
public class AutoTestLand extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        land();
    }
}
