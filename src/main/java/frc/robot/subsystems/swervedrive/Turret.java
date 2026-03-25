// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  TalonFX turret = new TalonFX(Constants.TurretID);
  PositionVoltage PIDTurret = new PositionVoltage(0);

  SwerveSubsystem swerve;

  //For configurations
  VoltageOut turretVoltage = new VoltageOut(0);

  //Target
  double targetTurretSetpoint = 0;

  Aimmer aimmer;

  public enum turretCalibrated{
    NOT_CALIBRATED,
    CALIBRATED
  }

  public turretCalibrated calibrated = turretCalibrated.CALIBRATED; //NOT_CALIBRATED WHEN SENSOR

  /** Creates a new Turret. */
  public Turret(SwerveSubsystem swerveSubsystem, Aimmer a) {

    aimmer = a;

    swerve = swerveSubsystem;
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    turretConfig.Slot0.kP = 0.3;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;

    turretConfig.CurrentLimits.StatorCurrentLimit = 40;
    turretConfig.CurrentLimits.SupplyCurrentLimit = 40;

    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;
    turretConfig.ClosedLoopGeneral.ContinuousWrap = false;

    turretConfig.Voltage.PeakForwardVoltage = 2;
    turretConfig.Voltage.PeakReverseVoltage = -2;

    turretConfig.MotorOutput.PeakForwardDutyCycle = 0.01;
    turretConfig.MotorOutput.PeakReverseDutyCycle = -0.01;


    //turretConfig.MotorOutput.Inverted = 

    //NOT SET RIGHT NOW
    turretConfig.Feedback.SensorToMechanismRatio = Constants.turretConversionFactor;
    turretConfig.Feedback.RotorToSensorRatio = 1;

    turret.getConfigurator().apply(turretConfig);

  }

  // /**Idk why you would want to ff here? */
  // public void setReference(double setpoint, double ff) {
  //   if (calibrated == turretCalibrated.CALIBRATED) {
  //     turret.setControl(PIDTurret.withPosition(aimmer.inBounds(setpoint)).withVelocity(ff));
  //     targetTurretSetpoint = aimmer.inBounds(setpoint);
  //   }
  // }

  public void setDegreesReference(double degrees, double ff) {
    if (calibrated == turretCalibrated.CALIBRATED) {
      targetTurretSetpoint = aimmer.inBounds(degrees);
      turret.setControl(PIDTurret.withPosition(targetTurretSetpoint).withVelocity(ff));
      //System.out.println("DEGRESSWORKS");
    }
  }

  public double getTargetTurretSetpoint() {
    return targetTurretSetpoint ;
  }

  public double getDistanceToRealHub() {
    return aimmer.getDistanceTurretToRealHub();
  }

  public double getDistanceToVirtualHub() {
    return aimmer.getDistanceTurretToVirtualHub();
  }

  public void setVoltage(double volts) {
    turret.setControl(turretVoltage.withOutput(volts));
  }

  public Aimmer getAimmer() {
    return aimmer;
  }

  public void setEncoder(double value) {
    turret.setPosition(value);
  }

  public void setCalibrated(turretCalibrated tc) {
    calibrated = tc;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turretPIDTargetDegrees", targetTurretSetpoint);
    SmartDashboard.putNumber("turretPIDTargetROtations", targetTurretSetpoint * Constants.turretConversionFactor);
    SmartDashboard.putNumber("distance", getDistanceToRealHub());
    
    swerve.getField().getObject("turretmanual?").setPose(new Pose2d(aimmer.turretPose.getTranslation(), aimmer.turretPose.getRotation().rotateBy(new Rotation2d(-targetTurretSetpoint * (Math.PI / 180)))));

    SmartDashboard.putNumber("TurretRelativeAngle", turret.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("inBounds", aimmer.getInBounds());
    SmartDashboard.putNumber("TurretVoltage", turret.getMotorVoltage().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
