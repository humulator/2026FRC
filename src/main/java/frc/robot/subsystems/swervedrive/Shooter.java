// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX shooter = new TalonFX(Constants.ShooterID);
  VelocityVoltage velocityPIDShooter = new VelocityVoltage(0);
  VoltageOut voltageShooter = new VoltageOut(0);

  Aimmer aimmer;

  double manualTargetRotationsPerSecond = 35;
  double manualTargetVoltage = 0;

  /** Creates a new Shooter. */
  public Shooter(Aimmer a) {

    aimmer = a;

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    shooterConfig.CurrentLimits.StatorCurrentLimit = 40;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.Slot0.kP = 0.1; //0.2
    shooterConfig.Slot0.kI = 0.00;
    shooterConfig.Slot0.kD = 0.00;
    shooterConfig.Slot0.kV = 0.12;

    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // RampRate
    shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;


    shooter.getConfigurator().apply(shooterConfig);

  }

  public boolean isCloseEnough() {
    double rps = shooter.getVelocity().getValueAsDouble();
    //manualTargetRotationsPerSecond
    return MathUtil.isNear(manualTargetRotationsPerSecond, rps, 2);
  }

  /**METERS */
  public double getSpeedFromDistance(double meters){
    return aimmer.speedFromDistance.get(meters);
  }
  
  /**METERS */
  public double getVoltageFromDistance(double meters) {
    return aimmer.voltageFromDistance.get(meters);
  }

  /**Rotations per second ACTUALLY TURNS*/
  public void setSpeedClosedLoop(double rotationsPerSecond) {
    shooter.setControl(velocityPIDShooter.withVelocity(rotationsPerSecond));
  }

  /** ACTUALLY TURSNS */
  public void setVoltageOpenLoop(double volts) {
    shooter.setControl(voltageShooter.withOutput(volts));
  }

  /** ONLY THE TARGET NOT TURNS */
  public void setTargetManualVoltage(double volts) {

    if (volts > Constants.maxShooterVoltage) {
      volts = Constants.maxShooterVoltage;
    }

    if (volts < Constants.minShooterVoltage) {
      volts = Constants.minShooterVoltage;
    }

    manualTargetVoltage = volts;
  }

  /**ONLY THE TARGET DOESNT TURN IT */
  public void setTargetManualRPS(double RPS) {

    if (RPS > Constants.maxShooterRPS) {
      RPS = Constants.maxShooterRPS;
    }

    if (RPS < Constants.minShooterRPS) {
      RPS = Constants.minShooterRPS;
    }

    manualTargetRotationsPerSecond = RPS;
  }

  public double getTargetManualVoltage() {
    return manualTargetVoltage;
  }

  public double getTargetManualRPS() {
    return manualTargetRotationsPerSecond;
  }

  public String getTheCurrentCommand() {
    if (getCurrentCommand() == null) {
      return "nothing";
    } else {
      return getCurrentCommand().getName();
    }
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("actual shooterRPS direct", shooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("targetRPS", getTargetManualRPS());
    SmartDashboard.putString("CommandRunningShooter", getTheCurrentCommand());
    SmartDashboard.putBoolean("ShooterisShooterRPSCloseEnoughToTarget", isCloseEnough());
    SmartDashboard.putNumber("getDistanceFromRPSManual", aimmer.getDistanceFromSpeed(manualTargetRotationsPerSecond));
    // This method will be called once per scheduler run
  }
}
