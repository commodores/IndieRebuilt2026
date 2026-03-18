// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
/*import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShooterRpmTable;*/


public class Shooter extends SubsystemBase {
  private static final double SECONDS_PER_MINUTE = 60.0;
  private static final double SHOOTER_COMMAND_ON_THRESHOLD_RPM = 100.0;
  private static final double SHOOTER_AT_SPEED_TOLERANCE_RPM = 200.0;

  private final SparkFlex leftTopLeader = new SparkFlex(ShooterConstants.shooterTopLeft);
  private final SparkFlex leftBottomFollower = new SparkFlex(ShooterConstants.shooterBottomLeft);
  private final SparkFlex rightTopLeader = new SparkFlex(ShooterConstants.shooterTopRight);
  private final SparkFlex rightBottomFollower = new SparkFlex(ShooterConstants.shooterBottomRight);

  private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private double targetLeftRpm = 0.0;
  private double targetRightRpm = 0.0;

  
  private final VoltageOut sysIdVoltage = new VoltageOut(0);

  private final SysIdRoutine sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,  // default ramp rate (1 V/sec is fine)
            Volts.of(7),  // step voltage limit (7V is good for shooter)
            null
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> runShooterVoltage(volts.in(Volts)),
            log -> {
              // Phoenix 6 signals (rotations and rotations/sec)
              double posRot = leftTopLeader.getPosition().getValueAsDouble();
              double velRps = leftTopLeader.getVelocity().getValueAsDouble();
              double motV   = leftTopLeader.getMotorVoltage().getValueAsDouble();

              log.motor("shooter")
                 .voltage(Volts.of(motV))
                 .angularPosition(Rotations.of(posRot))
                 .angularVelocity(RotationsPerSecond.of(velRps));
            },
            this
        )
    );
  public Shooter() {
    leftTopLeader.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    leftBottomFollower.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    rightTopLeader.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    rightBottomFollower.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());



   // leftTopLeader.getConfigurator().apply(Configs.ShooterSubsystem.shooterLeftTopConfig);
    //leftBottomFollower.getConfigurator().apply(Configs.ShooterSubsystem.shooterLeftBottomConfig);
   // rightTopLeader.getConfigurator().apply(Configs.ShooterSubsystem.shooterRightTopConfig);
    //rightBottomFollower.getConfigurator().apply(Configs.ShooterSubsystem.shooterRightBottomConfig);

    leftBottomFollower.setControl(
        new Follower(leftTopLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightBottomFollower.setControl(
        new Follower(rightTopLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }
  public void runShooterVoltage(double volts) {
      leftTopLeader.setControl(sysIdVoltage.withOutput(volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.dynamic(direction);
  }
  
  
  public void setShooterSpeedRpm(double rpm) {
    setShooterSpeedRpm(rpm, rpm);
  }

  public void setShooterSpeedRpm(double leftRpm, double rightRpm) {
    targetLeftRpm = leftRpm;
    targetRightRpm = rightRpm;
    // TalonFX velocity setpoints are in rotations per second.
    leftTopLeader.setControl(leftVelocityRequest.withVelocity(leftRpm / SECONDS_PER_MINUTE));
    rightTopLeader.setControl(rightVelocityRequest.withVelocity(rightRpm / SECONDS_PER_MINUTE));
  }

  public double getLeftShooterRpm() {
    return leftTopLeader.getVelocity().getValueAsDouble() * SECONDS_PER_MINUTE;
  }

  public double getRightShooterRpm() {
    return rightTopLeader.getVelocity().getValueAsDouble() * SECONDS_PER_MINUTE;
  }

  public double getTargetLeftRpm() {
    return targetLeftRpm;
  }

  public double getTargetRightRpm() {
    return targetRightRpm;
  }

  public boolean isCommandedOn() {
    return targetLeftRpm > SHOOTER_COMMAND_ON_THRESHOLD_RPM
        && targetRightRpm > SHOOTER_COMMAND_ON_THRESHOLD_RPM;
  }

  public boolean isAtSpeed() {
    return isCommandedOn()
        && Math.abs(getLeftShooterRpm() - targetLeftRpm) <= SHOOTER_AT_SPEED_TOLERANCE_RPM
        && Math.abs(getRightShooterRpm() - targetRightRpm) <= SHOOTER_AT_SPEED_TOLERANCE_RPM;
  }

  public double getShooterRpmForDistance(double distanceMeters) {
    ShooterRpmTable.Rpm target = ShooterRpmTable.getTarget(distanceMeters);
    return (target.left() + target.right()) / 2.0;
  }

  public void coastShooter() {
    targetLeftRpm = 0.0;
    targetRightRpm = 0.0;
    // Let the shooter spin down naturally instead of commanding a hard velocity target.
    leftTopLeader.setControl(neutralRequest);
    rightTopLeader.setControl(neutralRequest);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/ActualRPMLeft", getLeftShooterRpm());
    SmartDashboard.putNumber("Shooter/ActualRPMRight", getRightShooterRpm());
    SmartDashboard.putNumber("Shooter/TargetRPMLeft", targetLeftRpm);
    SmartDashboard.putNumber("Shooter/TargetRPMRight", targetRightRpm);
    SmartDashboard.putBoolean("Shooter/CommandedOn", isCommandedOn());
    SmartDashboard.putBoolean("Shooter/AtSetpoint", isAtSpeed());
  }
}
