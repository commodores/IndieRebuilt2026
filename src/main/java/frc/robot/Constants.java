
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Constants {

    public static final class Debug {
        // Set true to expose advanced bindings used for tuning and diagnostics.
        public static final boolean ENABLE_SYSID_BINDINGS = false;
        public static final boolean ENABLE_TEST_BINDINGS = false;
    }

    public static final class IntakeConstants{
        public static final int intakeMotor = 1;
        public static final int intakeLiftRight = 2;
        public static final int intakeLiftLeft = 3;
        public static final int intakeStowedTofCanId = 20;
        public static final int intakeDeployedTofCanId = 21;
        public static final double stowAngle = 90;
        public static final double deployAngle = -4;
        public static final double feedWiggleUpAngle = deployAngle + (stowAngle - deployAngle) * 0.5;
        public static final double feedWiggleStartDelaySec = .5;
        public static final double feedWiggleUpHoldSec = 0.5;
        public static final double feedWiggleDownHoldSec = 0.5;
        public static final double feedWiggleRollerVolts = -5.0;
        public static final double feedWiggleRollerPulseOnSec = 0.15;
        public static final double feedWiggleRollerPulseOffSec = 0.25;

        // Intake lift stuck-detection encoder correction tuning
        public static final double liftLimitDebounceSec = 0.05;
        public static final double liftGoalToleranceDeg = .5;
        public static final double tofProximityThresholdMm = 50.0;
        public static final double liftMoveWaitTimeoutSec = 1.5;
        public static final double feedWiggleMinSwitchDelaySec = 0.10;

        public static final double intakeDegreesPerRotation = 15.0;
        public static final double gearRatio = 24.0;    
    }

    public static final class ShooterConstants{
        public static final int shooterTopLeft = 4;
        public static final int shooterBottomLeft = 5;
        public static final int shooterTopRight = 6;
        public static final int shooterBottomRight = 7;

        // Vision-aim shooter RPM behavior.
        public static final double visionFallbackRpm = 4250.0;
        public static final double visionMaxTargetRpm = 5500.0;
        public static final double noTargetHoldTimeoutSeconds = 2.0;
    }

   public static final class FeederConstants{
        public static final int rollerMotor = 8;
        public static final int bottomMotor = 7;
        public static final int topMotor = 9;
        
        
        public static final double bottomClearOutSeconds = 0.2;
        public static final double bottomSpinupDelaySeconds = 0.25;

        public static final double topSpinupDelaySeconds = 0.2;
        public static final double topClearOutSeconds = 0.25;
        
        public static final double feederRunTimedSeconds = 4;

        public static final double bottomFeedVolts = 12.0;
        public static final double topFeedVolts = 12.0;
        public static final double rollerFeedVolts = 10.0;
        
        public static final double bottomClearReverseVolts = -4.0;`
        public static final double topClearReverseVolts = -4.0;
        public static final double beltClearReverseVolts = -4.0;
       
        public static final double bottomSpinupVolts = 8.0;
        public static final double topSpinupVolts = 8.0;
        public static final double rollerOffVolts = 0.0;

        // Mechanism positive direction relative to motor positive rotation.
        // +1.0 means same direction, -1.0 means opposite direction.
        public static final double rollerMotorDirectionScalar = -1.0;
        public static final double bottomMotorDirectionScalar = 1.0;
        public static final double topMotorDirectionScalar = 1.0;
    }

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        public static final int PIPE_TAGS = 0;
        public static final int PIPE_RETRO = 1;
        public static final int PIPE_DRIVER = 2;

        // Camera geometry used for trig-only distance estimation.
        public static final double CAMERA_HEIGHT_M = 0.54;
        public static final double TARGET_HEIGHT_M = 1.4859; // AprilTag center height (m)
        public static final double CAMERA_PITCH_RAD = Math.toRadians(20.0);

        // Robot-to-camera transform (forward, left, up) in meters and roll/pitch/yaw in radians.
        public static final Transform3d ROBOT_TO_LIMELIGHT = new Transform3d(
            new Translation3d(0.30, 0.00, 0.20),
            new Rotation3d(0.0, 0.0, 0.0)
        );
    }


}
