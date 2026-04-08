// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  TalonFX feeder = new TalonFX(Constants.FeederID);
  VelocityVoltage velocityPIDFeeder = new VelocityVoltage(0);
  VoltageOut voltageFeeder = new VoltageOut(0);

  public enum feederState{
    OFF,
    ON,
    REVERSE
  }

  feederState fState = feederState.OFF;

  /** Creates a new Feeder. */
  public Feeder() {
TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    feederConfig.CurrentLimits.StatorCurrentLimit = 40;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    feederConfig.Slot0.kP = 0.4;
    feederConfig.Slot0.kI = 0.00;
    feederConfig.Slot0.kD = 0.00;

    feederConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;


    feeder.getConfigurator().apply(feederConfig);
  }

  /**Rotations per second */
  public void setSpeedClosedLoop(double rotationsPerSecond) {
    feeder.setControl(velocityPIDFeeder.withVelocity(rotationsPerSecond));
  }

  // public void setVoltageOpenLoop(double volts) {
  //   feeder.setControl(voltageFeeder.withOutput(volts));
  // }

  public void setVoltageOpenLoop(double volts, feederState s) {
    feeder.setControl(voltageFeeder.withOutput(volts));
    fState = s;
  }

  public feederState getFeederState() {
    return fState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FeederRPS", feeder.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
