package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.function.FloatSupplier;

public final class Configs {

  public static final class IntakeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig intakeLiftRightConfig = new SparkFlexConfig();   // leader
    public static final SparkFlexConfig intakeLiftLeftConfig = new SparkFlexConfig();  // follower

    static {

      // ----------------
      // Intake roller motor
      // ----------------
      //intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      //neo
      intakeConfig.idleMode(IdleMode.kCoast);
      intakeConfig.smartCurrentLimit(80);
      intakeConfig.secondaryCurrentLimit(110);

      //intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    //  intakeConfig.CurrentLimits.SupplyCurrentLimit = 80; // starter (A)
     // intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 110;
     // intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0.4;

      //intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
     // intakeConfig.CurrentLimits.StatorCurrentLimit = 110; // starter (A)

      // ----------------
      // Intake lift Right
      // ----------------

      //neo 
      intakeLiftRightConfig.inverted(false);
      intakeLiftRightConfig.idleMode(IdleMode.kBrake);
      intakeLiftRightConfig.smartCurrentLimit(60);
      intakeLiftRightConfig.secondaryCurrentLimit(90);

      //intakeLiftRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //intakeLiftRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      //intakeLiftRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      //intakeLiftRightConfig.CurrentLimits.SupplyCurrentLimit = 60; // increased for added belt tensioner load
      //intakeLiftRightConfig.CurrentLimits.SupplyCurrentLowerLimit = 90;
      //intakeLiftRightConfig.CurrentLimits.SupplyCurrentLowerTime = 0.3;

     // intakeLiftRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      //intakeLiftRightConfig.CurrentLimits.StatorCurrentLimit = 100;

      intakeLiftRightConfig.Feedback.SensorToMechanismRatio = 24.0;

      // Motion Magic + gains (your starters)
      intakeLiftRightConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      intakeLiftRightConfig.Slot0.kP = 100.0;
      intakeLiftRightConfig.Slot0.kI = 0.0;
      intakeLiftRightConfig.Slot0.kD = 1.0;

      intakeLiftRightConfig.Slot0.kG = 0.19212;
      intakeLiftRightConfig.Slot0.kS = 0.15156;
      intakeLiftRightConfig.Slot0.kV = 2.2577; //0.541; //2.0649
      intakeLiftRightConfig.Slot0.kA = 0.35577; //0.0585; //0.2235

      intakeLiftRightConfig.MotionMagic.MotionMagicCruiseVelocity = 4.0;
      intakeLiftRightConfig.MotionMagic.MotionMagicAcceleration   = 6.0;
      intakeLiftRightConfig.MotionMagic.MotionMagicJerk           = 15.0;

      intakeLiftRightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      intakeLiftRightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.28;

      intakeLiftRightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      intakeLiftRightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.02;
      
      // ----------------
      // Intake lift Left
      // Apply same safety limits + current limits
      // ----------------

      //neo 
      intakeLiftLeftConfig.idleMode(IdleMode.kBrake);
      intakeLiftLeftConfig.inverted(true);
      intakeLiftLeftConfig.smartCurrentLimit(60);
      intakeLiftLeftConfig.secondaryCurrentLimit(90);

      //intakeLiftLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //intakeLiftLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

     // intakeLiftLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
     // intakeLiftLeftConfig.CurrentLimits.SupplyCurrentLimit = 60;
     // intakeLiftLeftConfig.CurrentLimits.SupplyCurrentLowerLimit = 90;
    //  intakeLiftLeftConfig.CurrentLimits.SupplyCurrentLowerTime = 0.3;

     // intakeLiftLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      //intakeLiftLeftConfig.CurrentLimits.StatorCurrentLimit = 100;

      intakeLiftLeftConfig.Feedback.SensorToMechanismRatio = 24.0;

      // Motion Magic + gains (your starters)
      intakeLiftLeftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      intakeLiftLeftConfig.Slot0.kP = 100.0;
      intakeLiftLeftConfig.Slot0.kI = 0.0;
      intakeLiftLeftConfig.Slot0.kD = 1.0;

      intakeLiftLeftConfig.Slot0.kG = 0.19212;
      intakeLiftLeftConfig.Slot0.kS = 0.15156;
      intakeLiftLeftConfig.Slot0.kV = 2.2577; //0.541; //2.0649
      intakeLiftLeftConfig.Slot0.kA = 0.35577; //0.0585; //0.2235

      intakeLiftLeftConfig.MotionMagic.MotionMagicCruiseVelocity = 4.0;
      intakeLiftLeftConfig.MotionMagic.MotionMagicAcceleration   = 6.0;
      intakeLiftLeftConfig.MotionMagic.MotionMagicJerk           = 15.0;

      intakeLiftLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      intakeLiftLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.28;

      intakeLiftLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      intakeLiftLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.02;
   
    }
  }

  public static final class ShooterSubsystem {
    public static final TalonFXConfiguration shooterLeftTopConfig = new TalonFXConfiguration();      // leader
    //public static final TalonFXConfiguration shooterLeftBottomConfig = new TalonFXConfiguration();   // follower
    public static final TalonFXConfiguration shooterRightTopConfig = new TalonFXConfiguration();     // leader
    //public static final TalonFXConfiguration shooterRightBottomConfig = new TalonFXConfiguration();  // follower
    //neo 
    public static final SparkFlexConfig

    static {
      // ----------------
      // LEFT side (top leader + bottom follower)
      // ----------------
      shooterLeftTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      //shooterLeftBottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      // Same-direction follower behavior -> keep inversion aligned on top + bottom
      shooterLeftTopConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      //shooterLeftBottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      shooterLeftTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      shooterLeftTopConfig.CurrentLimits.SupplyCurrentLimit = 80;
      shooterLeftTopConfig.CurrentLimits.SupplyCurrentLowerLimit = 100;
      shooterLeftTopConfig.CurrentLimits.SupplyCurrentLowerTime = 0.75;
      shooterLeftTopConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      shooterLeftTopConfig.CurrentLimits.StatorCurrentLimit = 140;

      shooterLeftTopConfig.Slot0.kP = 0.08;
      shooterLeftTopConfig.Slot0.kV = 0.11538;  
      shooterLeftTopConfig.Slot0.kS = 0.089681;
      shooterLeftTopConfig.Slot0.kA = 0;  

    
      //shooterLeftBottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      //shooterLeftBottomConfig.CurrentLimits.SupplyCurrentLimit = 80;
      //shooterLeftBottomConfig.CurrentLimits.SupplyCurrentLowerLimit = 100;
      //shooterLeftBottomConfig.CurrentLimits.SupplyCurrentLowerTime = 0.75;
      //shooterLeftBottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      //shooterLeftBottomConfig.CurrentLimits.StatorCurrentLimit = 140;

      // ----------------
      // RIGHT side (top leader + bottom follower)
      // ----------------
      shooterRightTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      //shooterRightBottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      // Mirror inversion from left side so positive velocity command spins wheels forward
      shooterRightTopConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      //shooterRightBottomConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      shooterRightTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      shooterRightTopConfig.CurrentLimits.SupplyCurrentLimit = 80;
      shooterRightTopConfig.CurrentLimits.SupplyCurrentLowerLimit = 100;
      shooterRightTopConfig.CurrentLimits.SupplyCurrentLowerTime = 0.75;
      shooterRightTopConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      shooterRightTopConfig.CurrentLimits.StatorCurrentLimit = 140;

      shooterRightTopConfig.Slot0.kP = 0.08;
      shooterRightTopConfig.Slot0.kV = 0.11538;  
      shooterRightTopConfig.Slot0.kS = 0.089681;
      shooterRightTopConfig.Slot0.kA = 0;  

      //shooterRightBottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      //shooterRightBottomConfig.CurrentLimits.SupplyCurrentLimit = 70;
      //shooterRightBottomConfig.CurrentLimits.SupplyCurrentLowerLimit = 100;
      //shooterRightBottomConfig.CurrentLimits.SupplyCurrentLowerTime = 0.75;
      //shooterRightBottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      //shooterRightBottomConfig.CurrentLimits.StatorCurrentLimit = 140;
    }
  }

  public static final class FeederSubsystem {
    public static final TalonFXConfiguration beltConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    static {
      beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      beltConfig.CurrentLimits.SupplyCurrentLimit = 35;
      beltConfig.CurrentLimits.SupplyCurrentLowerLimit = 45;
      beltConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
      beltConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      beltConfig.CurrentLimits.StatorCurrentLimit = 80;

      kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      kickerConfig.CurrentLimits.SupplyCurrentLimit = 60;
      kickerConfig.CurrentLimits.SupplyCurrentLowerLimit = 80;
      kickerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
      kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      kickerConfig.CurrentLimits.StatorCurrentLimit = 100;

    }
  }
}
