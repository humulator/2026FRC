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

public class Kickup extends SubsystemBase {

  TalonFX kickup = new TalonFX(Constants.kickupID);
  VelocityVoltage velocityPIDKickup = new VelocityVoltage(0);
  VoltageOut voltageKickup = new VoltageOut(0);

  /** Creates a new Kickup. */
  public Kickup() {
    TalonFXConfiguration kickupConfig = new TalonFXConfiguration();

    kickupConfig.CurrentLimits.StatorCurrentLimit = 40;
    kickupConfig.CurrentLimits.SupplyCurrentLimit = 40;
    kickupConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickupConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kickupConfig.Slot0.kP = 0.3;
    kickupConfig.Slot0.kI = 0.00;
    kickupConfig.Slot0.kD = 0.00;

    kickupConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickupConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;


    kickup.getConfigurator().apply(kickupConfig);
  }

   /**Rotations per second */
  public void setSpeedClosedLoop(double rotationsPerSecond) {
    kickup.setControl(velocityPIDKickup.withVelocity(rotationsPerSecond));
  }

  public void setVoltageOpenLoop(double volts) {
    kickup.setControl(voltageKickup.withOutput(volts));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("KickupRPS", kickup.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
