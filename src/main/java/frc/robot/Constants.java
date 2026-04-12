// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = 3.7;//Units.feetToMeters(10); 3.04m/s //14.5 (4.4m/s) defualt
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //lovely conversion factor we got here
  public static final double turretConversionFactor = 7.67 / 90; //rotations per degrees
  public static final double intakeArmConversionFactor = 1;

  //IDS
  public static final int TurretID = 30; //30 or "20"
  public static final int ShooterID = 31;//31 or "21"
  public static final int FeederID = 32;
  public static final int IntakeID = 33;
  public static final int intakeRoller = 34;
  public static final int kickupID = 35;

  //turret setpoints DEGREES
public static final double maxTurretSetpoint = 105;
public static final double minTurretSetpoint = -75;

public static final double turretTooCloseMeters = 2.2;

public static final double maxShooterRPS = 90;
public static final double minShooterRPS = 35;

//THESE ARE NOT USED
public static final double maxShooterVoltage = 0.01;
public static final double minShooterVoltage = -0.01;

//Speeds
public static final double standardFeederSpeed = 55;
public static final double kickupStandardSpeed = 55;

public static final double intakeUpSetpoint = -0.2; //with manual pid 108
public static final double intakeDownSetpoint = 16; //with manual pid -34
public static final double intakeBobSetpoint = 8;

public static final double intakeTickerNumber = 20;

// speeds
public static final double IntakeForwardSpeed = -12;
public static final double IntakeBackwardSpeed = 12;

// Turret Calibration Values
public static final int twentymsTicks = 100;
public static final double turretSensorCalibrationValue = 0;
public static final double turretCalibratedValue = 0;
public static final double turretVoltageForCalibration = 1;
public static final double LEDWarningSeconds = 3;

  

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


}
