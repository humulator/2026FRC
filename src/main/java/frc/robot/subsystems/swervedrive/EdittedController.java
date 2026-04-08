// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class EdittedController extends SubsystemBase {
  public CommandXboxController controller;
  public double offset = 0;

  public double factor = 1;

  public enum driveSpeedState{
    normal,
    slow
  }

  driveSpeedState drivespeed = driveSpeedState.normal;

  /** Creates a new EdittedController. */
  public EdittedController(int id){
    controller = new CommandXboxController(id);
  }

  public double getX() {
    double theta;
    double mag;

    mag = Math.sqrt((controller.getLeftX() * controller.getLeftX()) + (controller.getLeftY() * controller.getLeftY()));
    theta = Math.atan2(controller.getLeftY(), controller.getLeftX());
    theta += offset;

    return Math.cos(theta) * mag * factor;
  }

  public double getY() {
    double theta;
    double mag;

    mag = Math.sqrt((controller.getLeftX() * controller.getLeftX()) + (controller.getLeftY() * controller.getLeftY()));
    theta = Math.atan2(controller.getLeftY(), controller.getLeftX());
    theta += offset;

    return Math.sin(theta) * mag * factor;
  }

  public void setOffset(double deg) {
    offset = Units.degreesToRadians(-deg);
  }

  public void setFactor(double fac, driveSpeedState s) {
    factor = fac;
    drivespeed = s;
  }

  public driveSpeedState getSpeedState() {
    return drivespeed;
  }

  public double getInvertedRightY() {
    return -controller.getRightY();
  }

  public double getInvertedRightX() {
    return -controller.getRightX();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}