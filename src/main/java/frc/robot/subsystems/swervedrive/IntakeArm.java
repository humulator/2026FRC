// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {

  //DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
  //PIDController wpilibPID = new PIDController(0.03, 0, 0);

  TalonFX intakeArm = new TalonFX(Constants.IntakeID);
  PositionVoltage PIDIntakeArm = new PositionVoltage(0);

  double curSetpoint = 0;

  public enum IntakeArmState {
    NONE,
    EXTENDED,
    RETRACTED
  }

  IntakeArmState intakeArmState = IntakeArmState.NONE;
 

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    TalonFXConfiguration intakeArmConfig = new TalonFXConfiguration();

    intakeArmConfig.Slot0.kP = 0.4;
    intakeArmConfig.Slot0.kI = 0;
    intakeArmConfig.Slot0.kD = 0;

    intakeArmConfig.CurrentLimits.StatorCurrentLimit = 20;
    intakeArmConfig.CurrentLimits.SupplyCurrentLimit = 20;

    intakeArmConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeArmConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeArmConfig.Voltage.PeakForwardVoltage = 3;
    intakeArmConfig.Voltage.PeakReverseVoltage = -3;

    intakeArmConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;

    intakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //intakeArmConfig.MotorOutput.Inverted = 

    //NOT SET RIGHT NOW
    intakeArmConfig.Feedback.SensorToMechanismRatio = Constants.intakeArmConversionFactor;
    intakeArmConfig.Feedback.RotorToSensorRatio = 1;

    intakeArm.getConfigurator().apply(intakeArmConfig);

  }

  // public double getContinousRotations() {
  //   return absEncoder.get() + 0;
  // }

  // public double getDegreesAngleToHorizontal() {
  //   return -(Units.rotationsToDegrees(getContinousRotations() + 0) - 210);
  // }

  // public void seedAbsToRel() {
  //   intakeArm.setPosition(absEncoder.get());
  // }

  public void setReference(double setpoint, double ff) {
    intakeArm.setControl(PIDIntakeArm.withPosition(setpoint).withVelocity(ff));
    curSetpoint = setpoint;
  }

  //VoltageOut controlVoltage = new VoltageOut(0); 
  // public void usewpilibPID() {
  //   double feedforward = -(Math.cos(Units.degreesToRadians(getDegreesAngleToHorizontal())) * 0.005); //ff
  //   double pid = -(wpilibPID.calculate(getDegreesAngleToHorizontal(), curSetpoint));

  //   double calculatedVoltage = pid + feedforward;

  //   if (calculatedVoltage > 1) {
  //     calculatedVoltage = 1;
  //   }
  //   if (calculatedVoltage < -1) {
  //     calculatedVoltage = -1;
  //   }

  //   controlVoltage.withOutput(calculatedVoltage);

  //   SmartDashboard.putNumber("ArmCalculatedVoltage", controlVoltage.Output);
  //   SmartDashboard.putNumber("ArmPIDCalculated", pid);
  //   SmartDashboard.putNumber("ArmFFCalucluated", feedforward);
  //   SmartDashboard.putNumber("ArmSetpoint", curSetpoint);

  //   intakeArm.setControl(controlVoltage);
  //   //intakeArm.setControl(new VoltageOut(0));
  //   }

  // public void setSetpoint(double setpoint) {
  //   curSetpoint = setpoint;
  // }

  // public void setReference(double setpoint, double ff, IntakeArmState state) {
  //   intakeArm.setControl(PIDIntakeArm.withPosition(setpoint).withVelocity(ff));
  //   intakeArmState = state;
  // }

  public IntakeArmState getIntakeArmState() {
    return intakeArmState;
  }

  public boolean isNearTarget(double setpoint) {

    return MathUtil.isNear(setpoint, intakeArm.getPosition().getValueAsDouble(), 2);
  }

  @Override
  public void periodic() {
    //usewpilibPID();
    SmartDashboard.putNumber("ArmintakeArmRelative", intakeArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ArmVoltage", intakeArm.getMotorVoltage().getValueAsDouble());
    //SmartDashboard.putNumber("ArmIntakeArmAbsoluteValue", absEncoder.get());
    //SmartDashboard.putNumber("ArmintakeAngleToHorizontal", getDegreesAngleToHorizontal());
    //SmartDashboard.putString("IntakeArmState", intakeArmState.toString());
    // This method will be called once per scheduler run
  }
}
