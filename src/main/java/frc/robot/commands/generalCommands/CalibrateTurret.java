// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Aimmer;
import frc.robot.subsystems.swervedrive.Turret;
import frc.robot.subsystems.swervedrive.Turret.turretCalibrated;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateTurret extends Command {

  Aimmer aimmer;
  Turret turret;  
  /** Creates a new CalibrateTurret. */
  public CalibrateTurret(Aimmer a, Turret t) {
    aimmer = a;
    turret = t;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  enum SensorChangeState{
    NOT_DETECTED,
    DECREASING,
    INCREASING,
    NOT_CHANGING
  }

  
  //double[] pastSensorStates = {10000000, 1000000, 100000000, 10000000};
  double last_sensor = 0;
  double curState = 0;
  double velocity = 0;
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  
  public SensorChangeState getDirectionOfChangeInSensor(){
    SensorChangeState sensorChangeState = SensorChangeState.NOT_DETECTED;
      curState = aimmer.getmmSensor();

      if (last_sensor == curState) {
        sensorChangeState = SensorChangeState.NOT_CHANGING;
      }
      if (last_sensor >= curState) {
        sensorChangeState = SensorChangeState.DECREASING;
      }
      if (last_sensor <= curState) {
        sensorChangeState = SensorChangeState.INCREASING;
      }
      if (curState >= 1000 || last_sensor >= 1000) {
          sensorChangeState = SensorChangeState.NOT_DETECTED;
      }
      return sensorChangeState;
  }

  int ticker = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (curState > Constants.turretSensorCalibrationValue) {
    //   if (getDirectionOfChangeInSensor() == SensorChangeState.NOT_CHANGING) {
    //     turret.setVoltage(Constants.turretVoltageForCalibration);
    //   }
    //   if (getDirectionOfChangeInSensor() == SensorChangeState.INCREASING) {
    //     turret.setVoltage(-Constants.turretVoltageForCalibration);
    //     ticker = 0;
    //   }
    //   if (getDirectionOfChangeInSensor() == SensorChangeState.DECREASING) {
    //     turret.setVoltage(Constants.turretVoltageForCalibration);
    //     ticker = 0;
    //   }
    //   if (getDirectionOfChangeInSensor() == SensorChangeState.NOT_DETECTED) {
    //     turret.setVoltage(Constants.turretVoltageForCalibration);
    //     ticker = 0;
    //   }
    // } else {
    //   turret.setVoltage(0);
    //   ticker += 1;
    // }

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("TURRET CALCIBRATION FAIKED");
      System.out.println("TURRET CALCIBRATION FAIKED");
      System.out.println("TURRET CALCIBRATION FAIKED");
    } else {
      turret.setEncoder(Constants.turretCalibratedValue);
      turret.setCalibrated(turretCalibrated.CALIBRATED);
    }
    turret.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ticker >= Constants.twentymsTicks);
  }
}
