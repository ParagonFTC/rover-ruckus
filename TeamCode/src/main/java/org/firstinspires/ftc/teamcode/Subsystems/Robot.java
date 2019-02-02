package org.firstinspires.ftc.teamcode.Subsystems;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

public abstract class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    public interface Listener {
        void onPostUpdate();
    }

    protected List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private List<CountDownLatch> cycleLatches;
    private OpModeManagerImpl opModeManager;
    private ExecutorService subsystemUpdateExecutor;

    private List<Listener> listeners;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            if (!subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.add(subsystem);
                            }
                        }
                    }
                }
                for (Listener listener : listeners) {
                    listener.onPostUpdate();
                }
                synchronized (cycleLatches) {
                    int i = 0;
                    while (i < cycleLatches.size()) {
                        CountDownLatch latch = cycleLatches.get(i);
                        latch.countDown();
                        if (latch.getCount() == 0) {
                            cycleLatches.remove(i);
                        } else {
                            i ++;
                        }
                    }
                }
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    public Robot(OpMode opMode) {
        listeners = new ArrayList<>();

        subsystems = new ArrayList<>();

        initHardware(opMode.hardwareMap);

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");

        subsystemsWithProblems = new ArrayList<>();
        RobotLog.registerGlobalWarningSource(this);

        cycleLatches = new ArrayList<>();
    }

    protected abstract void initHardware(HardwareMap hardwareMap);

    public void addListener(Listener listener) {
        listeners.add(listener);
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        RobotLog.unregisterGlobalWarningSource(this);
    }

    public void waitForNextCycle() {
        CountDownLatch latch = new CountDownLatch(1);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void waitOneFullCycle() {
        CountDownLatch latch = new CountDownLatch(2);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }

    @Override
    public String getGlobalWarning() {
        List<String> warnings = new ArrayList<>();
        synchronized (subsystemsWithProblems) {
            for (Subsystem subsystem : subsystemsWithProblems) {
                warnings.add("Problem with " + subsystem.getClass().getSimpleName());
            }
        }
        return RobotLog.combineGlobalWarnings(warnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblems) {
            subsystemsWithProblems.clear();
        }
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
