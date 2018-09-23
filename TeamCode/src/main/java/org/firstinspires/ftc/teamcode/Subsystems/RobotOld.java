package org.firstinspires.ftc.teamcode.Subsystems;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

public class RobotOld implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    public interface Listener {
        void onPostUpdate();
    }

    public FtcDashboard dashboard;

    //subsystems
    public TankDrive drive;

    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private List<CountDownLatch> cycleLatches;
    private OpModeManagerImpl opModeManager;
    private ExecutorService subsystemUpdateExecutor, telemetryUpdateExecutor;
    private BlockingQueue<TelemetryPacket> telemetryPacketQueue;

    private List<Listener> listeners;

    private Boolean started;

    private Runnable systemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            try {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        Map<String, Object> telemetry = subsystem.update(telemetryPacket.fieldOverlay());
                        telemetryPacket.putAll(telemetry);
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for" + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
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
                while (telemetryPacketQueue.remainingCapacity() == 0) {
                    Thread.sleep(1);
                }
                telemetryPacketQueue.add(telemetryPacket);
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

    private Runnable telemetryUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                TelemetryPacket packet = telemetryPacketQueue.take();
                dashboard.sendTelemetryPacket(packet);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    };

    public RobotOld(OpMode opMode) {
        dashboard = FtcDashboard.getInstance();

        listeners = new ArrayList<>();

        subsystems = new ArrayList<>();

        try {
            drive = new TankDrive(opMode.hardwareMap);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping TankDrive");
        }

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager =OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
        telemetryUpdateExecutor = ThreadPool.newSingleThreadExecutor("telemetry update");

        telemetryPacketQueue = new ArrayBlockingQueue<>(10);

        subsystemsWithProblems = new ArrayList<>();

        cycleLatches = new ArrayList<>();
    }

    public void addListener(Listener listener) {
        listeners.add(listener);
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(systemUpdateRunnable);
            telemetryUpdateExecutor.submit(telemetryUpdateRunnable);
            started = true;
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        if (telemetryUpdateExecutor != null) {
            telemetryUpdateExecutor.shutdownNow();
            telemetryUpdateExecutor = null;
        }
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
        return null;
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
