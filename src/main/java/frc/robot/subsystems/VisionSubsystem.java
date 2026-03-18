// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.OptionalDouble;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import java.util.OptionalDouble;

public class VisionSubsystem extends SubsystemBase {
    private static final double MIN_VALID_DISTANCE_METERS = 0.5;
    private static final double MAX_VALID_DISTANCE_METERS = 10.0;
    private static final double DISTANCE_HOLD_TIMEOUT_SECONDS = 0.75;
    private static final double DISTANCE_SLEW_RATE_METERS_PER_SECOND = 2.0;
    private double kP = 0.0;

    public enum DistanceSource {
        RAW,
        HELD,
        NONE
    }

    private final SlewRateLimiter distanceLimiter = new SlewRateLimiter(DISTANCE_SLEW_RATE_METERS_PER_SECOND);

    private double lastValidDistanceMeters = Double.NaN;
    private int lastValidTagId = -1;
    private double lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;

    private double heldDistanceMeters = Double.NaN;
    private double filteredDistanceMeters = Double.NaN;
    private boolean filterInitialized = false;
    private DistanceSource activeDistanceSource = DistanceSource.NONE;
    private double activeDistanceAgeSeconds = Double.POSITIVE_INFINITY;

    public VisionSubsystem() {
        setPipeline(VisionConstants.PIPE_TAGS);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
    }

    public double getTxDeg() {
        return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
    }

    public double getTyDeg() {
        return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
    }

    public int getFiducialId() {
        if (!hasTarget()) {
            return -1;
        }
        return (int) Math.round(LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME));
    }

    // Backwards-compatible alias for existing callers.
    public int getTagId() {
        return getFiducialId();
    }

    public void setPipeline(int id) {
        LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT_NAME, id);
    }

    public int getPipeline() {
        return (int) Math.round(LimelightHelpers.getCurrentPipelineIndex(VisionConstants.LIMELIGHT_NAME));
    }

    private OptionalDouble getCurrentValidDistanceMeters() {
        if (!hasTarget()) {
            return OptionalDouble.empty();
        }
        double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(VisionConstants.LIMELIGHT_NAME);
        if (targetPose == null || targetPose.length < 3) {
            return OptionalDouble.empty();
        }

        double xMeters = targetPose[0];
        double yMeters = targetPose[1];
        double zMeters = targetPose[2];
        double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);

        if (!Double.isFinite(distanceMeters)
                || distanceMeters < MIN_VALID_DISTANCE_METERS
                || distanceMeters > MAX_VALID_DISTANCE_METERS) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(distanceMeters);
    }

    public OptionalDouble getDistanceMeters() {
        return getCurrentValidDistanceMeters();
    }

    public double getDistanceMetersWithHold() {
        OptionalDouble currentDistance = getCurrentValidDistanceMeters();
        double nowSeconds = Timer.getFPGATimestamp();

        if (currentDistance.isPresent()) {
            double distanceMeters = currentDistance.getAsDouble();
            lastValidDistanceMeters = distanceMeters;
            lastValidTagId = getFiducialId();
            lastValidTimestampSeconds = nowSeconds;
            activeDistanceSource = DistanceSource.RAW;
            activeDistanceAgeSeconds = 0.0;
            return distanceMeters;
        }

        activeDistanceAgeSeconds = nowSeconds - lastValidTimestampSeconds;
        if (Double.isFinite(lastValidDistanceMeters)
                && activeDistanceAgeSeconds <= DISTANCE_HOLD_TIMEOUT_SECONDS) {
            activeDistanceSource = DistanceSource.HELD;
            return lastValidDistanceMeters;
        }

        activeDistanceSource = DistanceSource.NONE;
        return Double.NaN;
    }

    public double getFilteredDistanceMetersWithHold() {
        return filteredDistanceMeters;
    }

    public DistanceSource getActiveDistanceSource() {
        return activeDistanceSource;
    }

    public double getDistanceAgeSeconds() {
        return activeDistanceAgeSeconds;
    }

    public int getLastValidTagId() {
        return lastValidTagId;
    }

    public double getLastValidTimestampSeconds() {
        return lastValidTimestampSeconds;
    }

    public boolean hasDistanceEstimate() {
        return getDistanceMeters().isPresent();
    }
    

    @Override
    public void periodic() {
        boolean hasTarget = hasTarget();
        OptionalDouble rawDistance = getDistanceMeters();
        double rawDistanceMeters = rawDistance.isPresent() ? rawDistance.getAsDouble() : Double.NaN;

        heldDistanceMeters = getDistanceMetersWithHold();
        if (Double.isFinite(heldDistanceMeters)) {
            if (!filterInitialized) {
                distanceLimiter.reset(heldDistanceMeters);
                filteredDistanceMeters = heldDistanceMeters;
                filterInitialized = true;
            } else {
                filteredDistanceMeters = distanceLimiter.calculate(heldDistanceMeters);
            }
        }

        SmartDashboard.putBoolean("LL/HasTarget", hasTarget);
        SmartDashboard.putNumber("LL/TagID", getFiducialId());
        SmartDashboard.putNumber("LL/DistanceMetersRaw", rawDistanceMeters);
        SmartDashboard.putNumber("LL/DistanceMetersHeld", heldDistanceMeters);
        SmartDashboard.putNumber("LL/DistanceAgeSec", activeDistanceAgeSeconds);
        SmartDashboard.putBoolean("LL/UsingHeldDistance", activeDistanceSource == DistanceSource.HELD);
        SmartDashboard.putString("LL/DistanceSource", activeDistanceSource.name());
        SmartDashboard.putNumber("LL/DistanceMetersFiltered", filteredDistanceMeters);
        SmartDashboard.putBoolean("LL/DistanceFilterEnabled", true);
        SmartDashboard.putString("LL/Status", hasTarget ? "target" : "no target");
    }
}

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
