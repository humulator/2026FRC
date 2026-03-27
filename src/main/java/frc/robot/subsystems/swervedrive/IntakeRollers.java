// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {
  TalonFX intakeRoller = new TalonFX(Constants.intakeRoller);
  VelocityVoltage velocityPIDIntakeRollers = new VelocityVoltage(0);
  VoltageOut voltageIntakeRollers = new VoltageOut(0);

  public enum IntakeRollerState {
    ZERO,
    ON,
    REVERSE
  }

  IntakeRollerState intakeRollerState = IntakeRollerState.ZERO;

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    TalonFXConfiguration intakeRollersConfig = new TalonFXConfiguration();

    intakeRollersConfig.CurrentLimits.StatorCurrentLimit = 40;
    intakeRollersConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeRollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeRollersConfig.Slot0.kP = 0.001;
    intakeRollersConfig.Slot0.kI = 0.00;
    intakeRollersConfig.Slot0.kD = 0.00;

    intakeRollersConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;

    intakeRoller.getConfigurator().apply(intakeRollersConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intakeRollersRPS", intakeRoller.getVelocity().getValueAsDouble());
  }

  /**Rotations per second */
  public void setSpeedClosedLoop(double rotationsPerSecond) {
    intakeRoller.setControl(velocityPIDIntakeRollers.withVelocity(rotationsPerSecond));
  }

  /**Rotations per second */
  public void setSpeedClosedLoop(double rotationsPerSecond, IntakeRollerState state) {
    intakeRoller.setControl(velocityPIDIntakeRollers.withVelocity(rotationsPerSecond));
    intakeRollerState = state;
  }

  public void setVoltageOpenLoop(double volts) {
    intakeRoller.setControl(voltageIntakeRollers.withOutput(volts));
  }

}
