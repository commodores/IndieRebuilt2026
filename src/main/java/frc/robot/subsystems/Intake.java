// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.intakeMotor);
  private final SparkFlex intakeLiftRightMotor = new SparkFlex(IntakeConstants.intakeLiftRight);
  private final SparkFlex intakeLiftLeftMotor = new SparkFlex(IntakeConstants.intakeLiftLeft);

  private final MotionMagicVoltage rightLiftMM = new MotionMagicVoltage(0);
  private final MotionMagicVoltage leftLiftMM = new MotionMagicVoltage(0);
  private final VoltageOut intakeVoltageReq = new VoltageOut(0);

  
  private final SysIdRoutine liftSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds),
        Volts.of(1.5),
        Seconds.of(10.0)
      ),
      new SysIdRoutine.Mechanism(
          output -> liftVoltageOut(output.in(Volts)),
          log -> log.motor("intake-lift-right")
              .voltage(Volts.of(intakeLiftRightMotor.getMotorVoltage().getValueAsDouble()))
              .angularPosition(Radians.of(getLiftPositionArmRotations() * 2.0 * Math.PI))
              .angularVelocity(RadiansPerSecond.of(getLiftVelocityArmRps() * 2.0 * Math.PI)),
              //.angularPosition(Rotations.of(getLiftPositionArmRotations()))
              //.angularVelocity(RotationsPerSecond.of(getLiftVelocityArmRps())),
          this));
  

  private final DutyCycleOut intakePercentOutputRequest = new DutyCycleOut(0);
  private final DutyCycleOut liftStop = new DutyCycleOut(0);
  private final TimeOfFlight stowedTof = new TimeOfFlight(IntakeConstants.intakeStowedTofCanId);
  private final TimeOfFlight deployedTof = new TimeOfFlight(IntakeConstants.intakeDeployedTofCanId);

  private final Debouncer stowedDebounce = new Debouncer(IntakeConstants.liftLimitDebounceSec);
  private final Debouncer deployedDebounce = new Debouncer(IntakeConstants.liftLimitDebounceSec);

  private static final double snapWindowDeg = 6.0;

  private double goalDeg = IntakeConstants.stowAngle;
  private boolean stowedLimitRaw;
  private boolean deployedLimitRaw;
  private boolean stowedLimitDebounced;
  private boolean deployedLimitDebounced;
  private boolean didSnapDeploy;
  private boolean didSnapStow;
  private boolean snappedThisCycle;
  private boolean enableLimitSnaps = true;

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    intakeLiftLeftMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    intakeLiftRightMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

    intakeMotor.getConfigurator().apply(Configs.IntakeSubsystem.intakeConfig);
    intakeLiftLeftMotor.getConfigurator().apply(Configs.IntakeSubsystem.intakeLiftLeftConfig);
    intakeLiftRightMotor.getConfigurator().apply(Configs.IntakeSubsystem.intakeLiftRightConfig);

    setLiftEncoder(IntakeConstants.stowAngle);
  }

  public void setIntakePercentOutput(double percentOutput) {
    intakeMotor.setControl(intakePercentOutputRequest.withOutput(percentOutput));
  }

  public void setIntakeVoltage(double volts) {
    intakeMotor.setControl(intakeVoltageReq.withOutput(volts));
  }

  // --- Conversions ---
  private double rotationsToDegrees(double rotations) {
    return rotations * 360;
  }

  private double degreesToRotations(double degrees) {
    return degrees / 360;
  }

  public double getLiftPositionRotations() {
    return intakeLiftRightMotor.getPosition().getValueAsDouble();
  }

  public double getLiftPositionDegrees() {
    return rotationsToDegrees(getLiftPositionRotations());
  }

  public double getLiftPositionArmRotations() {
    return getLiftPositionRotations();
  }

  public double getLiftLeftPositionRotations() {
    return intakeLiftLeftMotor.getPosition().getValueAsDouble();
  }

  public double getLiftLeftPositionDegrees() {
    return rotationsToDegrees(getLiftLeftPositionRotations());
  }

  private double getLiftAverageDegrees() {
    return (getLiftPositionDegrees() + getLiftLeftPositionDegrees()) / 2.0;
  }

  public double getLiftVelocityArmRps() {
    // motor rps -> degrees/s -> arm rotations/s
    return intakeLiftRightMotor.getVelocity().getValueAsDouble();
  }

  public void setLiftEncoder(double angle) {
    double rotations = degreesToRotations(angle);
    intakeLiftRightMotor.setPosition(rotations);
    intakeLiftLeftMotor.setPosition(rotations);
  }

  public void moveLiftToDegrees(double targetDeg) {
    setGoalDegrees(targetDeg);
  }

  public void stowLift() {
    moveLiftToDegrees(IntakeConstants.stowAngle);
  }

  public void deployLift() {
    moveLiftToDegrees(IntakeConstants.deployAngle);
  }

  public boolean atGoalDeg(double toleranceDeg) {
    return Math.abs(goalDeg - getLiftAverageDegrees()) <= toleranceDeg;
  }

  public boolean atDown() {
    return Math.abs(getLiftAverageDegrees() - IntakeConstants.deployAngle)
        <= IntakeConstants.liftGoalToleranceDeg;
  }

  public boolean atUp() {
    return Math.abs(getLiftAverageDegrees() - IntakeConstants.stowAngle)
        <= IntakeConstants.liftGoalToleranceDeg;
  }

  public boolean stowedLimit() {
    return stowedLimitDebounced;
  }

  public boolean deployedLimit() {
    return deployedLimitDebounced;
  }

  public void setGoalDegrees(double goalDeg) {
    if (Math.abs(this.goalDeg - goalDeg) > 1e-6) {
      didSnapDeploy = false;
      didSnapStow = false;
    }
    this.goalDeg = goalDeg;
  }

  public void stopIntake(){
    setIntakePercentOutput(0.0);
  }

  /**
   * Deploys the intake, runs the roller while scheduled, and stops the roller when ended.
   * The intake remains deployed after the command finishes.
   */
  public Command deployAndRunCommand(double intakePercentOutput) {
    return Commands.startEnd(
        () -> {
          deployLift();
          setIntakePercentOutput(intakePercentOutput);
        },
        () -> setIntakePercentOutput(0.0),
        this);
  }

  public void liftVoltageOut(double volts) {
    goalDeg = getLiftAverageDegrees();
    intakeLiftRightMotor.setControl(new VoltageOut(volts));
    intakeLiftLeftMotor.setControl(new VoltageOut(volts));
  }

  public void setEnableLimitSnaps(boolean enable) {
    enableLimitSnaps = enable;
  }

  
  public Command sysIdLiftQuasistatic(SysIdRoutine.Direction direction) {
    return liftSysIdRoutine.quasistatic(direction)
        .beforeStarting(() -> setEnableLimitSnaps(false))
        .finallyDo((interrupted) -> setEnableLimitSnaps(true));
  }

  public Command sysIdLiftDynamic(SysIdRoutine.Direction direction) {
    return liftSysIdRoutine.dynamic(direction)
        .beforeStarting(() -> setEnableLimitSnaps(false))
        .finallyDo((interrupted) -> setEnableLimitSnaps(true));
  }
  

  @Override
  public void periodic() {
     double currentDeg = getLiftAverageDegrees();
     updateLimitStates();
     snappedThisCycle = false;
     if (enableLimitSnaps) {
       snappedThisCycle = maybeSnapEncoderToLimit(currentDeg);
     }
     if (snappedThisCycle) {
       currentDeg = getLiftAverageDegrees();
       publishTelemetry(currentDeg);
       return;
     }

     double rightDeg = getLiftPositionDegrees();
     double leftDeg = getLiftLeftPositionDegrees();
     double desyncDeg = leftDeg - rightDeg;
     boolean desyncFault = Math.abs(desyncDeg) > 8.0;
     boolean isMoving = Math.abs(goalDeg - currentDeg) > IntakeConstants.liftGoalToleranceDeg;
     if (desyncFault && isMoving) {
       intakeLiftRightMotor.setControl(liftStop);
       intakeLiftLeftMotor.setControl(liftStop);
       publishTelemetry(currentDeg);
       return;
     }

     // Always hold the current goal with Motion Magic.
     var targetRot = degreesToRotations(goalDeg);
     intakeLiftRightMotor.setControl(rightLiftMM.withPosition(targetRot).withSlot(0));
     intakeLiftLeftMotor.setControl(leftLiftMM.withPosition(targetRot).withSlot(0));

     publishTelemetry(currentDeg);

    // This method will be called once per scheduler run
  }

  private void publishTelemetry(double currentDeg) {
     double rightDeg = getLiftPositionDegrees();
     double leftDeg = getLiftLeftPositionDegrees();
     SmartDashboard.putNumber("Intake Lift Goal Deg", goalDeg);
     SmartDashboard.putNumber("Intake Lift Current Deg", currentDeg);
     SmartDashboard.putNumber("Intake Lift Right Deg", rightDeg);
     SmartDashboard.putNumber("Intake Lift Left Deg", leftDeg);
     double desyncDeg = leftDeg - rightDeg;
     SmartDashboard.putNumber("Intake Lift Left-Right Deg", desyncDeg);
     SmartDashboard.putBoolean("Intake Lift Desync Fault", Math.abs(desyncDeg) > 8.0);
     SmartDashboard.putBoolean("Intake Lift Deployed Limit Raw", deployedLimitRaw);
     SmartDashboard.putBoolean("Intake Lift Stowed Limit Raw", stowedLimitRaw);
     SmartDashboard.putBoolean("Intake Lift Deployed Limit Debounced", deployedLimitDebounced);
     SmartDashboard.putBoolean("Intake Lift Stowed Limit Debounced", stowedLimitDebounced);
     SmartDashboard.putNumber("Intake Lift Closed Loop Error", intakeLiftRightMotor.getClosedLoopError().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Duty Cycle", intakeLiftRightMotor.getDutyCycle().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Supply Current A", intakeLiftRightMotor.getSupplyCurrent().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Stator Current A", intakeLiftRightMotor.getStatorCurrent().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Left Closed Loop Error", intakeLiftLeftMotor.getClosedLoopError().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Left Duty Cycle", intakeLiftLeftMotor.getDutyCycle().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Left Supply Current A", intakeLiftLeftMotor.getSupplyCurrent().getValueAsDouble());
     SmartDashboard.putNumber("Intake Lift Left Stator Current A", intakeLiftLeftMotor.getStatorCurrent().getValueAsDouble());
     SmartDashboard.putBoolean("Intake Lift Snapped This Cycle", snappedThisCycle);
  }

  private void updateLimitStates() {
    stowedLimitRaw = readTofRaw(stowedTof);
    deployedLimitRaw = readTofRaw(deployedTof);
    stowedLimitDebounced = stowedDebounce.calculate(stowedLimitRaw);
    deployedLimitDebounced = deployedDebounce.calculate(deployedLimitRaw);
  }

  private boolean maybeSnapEncoderToLimit(double currentDeg) {
    if (deployedLimitDebounced
        && !didSnapDeploy
        && (isDeployGoal() || isMovingDownward(currentDeg))
        && Math.abs(currentDeg - IntakeConstants.deployAngle) <= snapWindowDeg) {
      setLiftEncoder(IntakeConstants.deployAngle);
      didSnapDeploy = true;
      return true;
    }
    if (stowedLimitDebounced
        && !didSnapStow
        && (isStowGoal() || isMovingUpward(currentDeg))
        && Math.abs(currentDeg - IntakeConstants.stowAngle) <= snapWindowDeg) {
      setLiftEncoder(IntakeConstants.stowAngle);
      didSnapStow = true;
      return true;
    }
    return false;
  }

  private boolean isDeployGoal() {
    return Math.abs(goalDeg - IntakeConstants.deployAngle) <= IntakeConstants.liftGoalToleranceDeg;
  }

  private boolean isStowGoal() {
    return Math.abs(goalDeg - IntakeConstants.stowAngle) <= IntakeConstants.liftGoalToleranceDeg;
  }

  private boolean isMovingDownward(double currentDeg) {
    return goalDeg < currentDeg - IntakeConstants.liftGoalToleranceDeg;
  }

  private boolean isMovingUpward(double currentDeg) {
    return goalDeg > currentDeg + IntakeConstants.liftGoalToleranceDeg;
  }

  private boolean readTofRaw(TimeOfFlight tofSensor) {
    if (!tofSensor.isRangeValid()) {
      return false;
    }
    double rangeMm = tofSensor.getRange();
    return rangeMm > 0.0 && rangeMm <= IntakeConstants.tofProximityThresholdMm;
  }
}
