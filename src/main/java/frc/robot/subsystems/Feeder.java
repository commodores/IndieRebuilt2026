// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;

public class Feeder extends SubsystemBase {

  public enum FeedState {
    CLEARING,
    KICKER_SPINUP,
    FEEDING,
    OFF
  }
 //change const 
  private final TalonFX rollerMotor = new TalonFX(FeederConstants.rollerMotor, "drivecan");
  //neos
  private final SparkFlex bottomMotor = new SparkFlex(FeederConstants.bottomMotor,MotorType.kBrushless);
  private final SparkFlex topMotor = new SparkFlex(FeederConstants.topMotor,MotorType.kBrushless);
  

  private final VoltageOut rollerVoltageReq = new VoltageOut(0);
  //neos
  private final VoltageOut bottomVoltageReq = new VoltageOut(0);
  private final VoltageOut topVoltageReq = new VoltageOut(0);

  private FeedState feedState = FeedState.OFF;

  /** Creates a new Feeder. */
  public Feeder() {
    rollerMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    //neos
    bottomMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    topMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

    rollerMotor.getConfigurator().apply(Configs.FeederSubsystem.rollerConfig);
    //neo
    bottomMotor.getConfigurator().apply(Configs.FeederSubsystem.kickerConfig);
    topMotor.getConfigurator().apply(Configs.FeederSubsystem.kickerConfig);
  }

  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(
        rollerVoltageReq.withOutput(volts * FeederConstants.rollerMotorDirectionScalar));
  }
  
 //Neo
  public void setTopVoltage(double volts) {
    topMotor.setControl(
        topVoltageReq.withOutput(volts * FeederConstants.topMotorDirectionScalar));
  }

  public void setBottomVoltage(double volts) {
    bottomMotor.setControl(
        bottomVoltageReq.withOutput(volts * FeederConstants.bottomMotorDirectionScalar));
  }

  public void setFeedState(FeedState state) {
    feedState = state;
  }

  public void stopFeeder() {
    feedState = FeedState.OFF;
    setRollerVoltage(0.0);
    //Neo?
    setTopVoltage(0.0);
    setBottomVoltage(0.0);
  }

  public String getFeedStateString() {
    return feedState.name();
  }

  @Override
  //Need imports for sparkmax
  public void periodic() {
    SmartDashboard.putNumber("Feeder/bottomAppliedVolts", bottomMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/RollerAppliedVolts", rollerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/TopAppliedVolts", topMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putString("Feeder/State", getFeedStateString());
  }
}
