// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

  public enum FeedState {
    CLEARING,
    KICKER_SPINUP,
    FEEDING,
    OFF
  }

  private final SparkFlex kickerMotor = new SparkFlex(FeederConstants.kickerMoter);
  private final SparkFlex beltMotor = new SparkFlex(FeederConstants.beltMotor);

  private final VoltageOut kickerVoltageReq = new VoltageOut(0);
  private final VoltageOut beltVoltageReq = new VoltageOut(0);

  private FeedState feedState = FeedState.OFF;

  /** Creates a new Feeder. */
  public Feeder() {
    beltMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    kickerMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

    beltMotor.getConfigurator().apply(Configs.FeederSubsystem.beltConfig);
    kickerMotor.getConfigurator().apply(Configs.FeederSubsystem.kickerConfig);
  }

  public void setKickerVoltage(double volts) {
    kickerMotor.setControl(
        kickerVoltageReq.withOutput(volts * FeederConstants.kickerMotorDirectionScalar));
  }

  public void setBeltVoltage(double volts) {
    beltMotor.setControl(
        beltVoltageReq.withOutput(volts * FeederConstants.beltMotorDirectionScalar));
  }

  public void setFeedState(FeedState state) {
    feedState = state;
  }

  public void stopFeeder() {
    feedState = FeedState.OFF;
    setKickerVoltage(0.0);
    setBeltVoltage(0.0);
  }

  public String getFeedStateString() {
    return feedState.name();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder/KickerAppliedVolts", kickerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/BeltAppliedVolts", beltMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putString("Feeder/State", getFeedStateString());
  }
}
