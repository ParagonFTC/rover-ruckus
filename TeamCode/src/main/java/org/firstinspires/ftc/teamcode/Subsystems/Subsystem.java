package org.firstinspires.ftc.teamcode.Subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.Map;

public abstract class Subsystem {
    public abstract Map<String, Object> update(@Nullable Canvas fieldOverlay);
}
