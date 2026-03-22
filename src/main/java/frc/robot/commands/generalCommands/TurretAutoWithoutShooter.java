// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAutoWithoutShooter extends Command {

  Turret turret;
  SwerveSubsystem swerve;
  Shooter shooter;
  /** Creates a new TurretAuto. */
  public TurretAutoWithoutShooter(Turret t, SwerveSubsystem s, Shooter ss) {
    turret = t;
    swerve = s;
    shooter = ss;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setDegreesReference(-turret.getAimmer().getAngleToRealHub().getDegrees(), 0);
    //shooter.setTargetManualRPS(shooter.getSpeedFromDistance(turret.getAimmer().getDistanceTurretToRealHub()));
    
    //turret.setReference(turret.getAimmer().getAngleToVirtualHub().getRadians(), 0);
    //shooter.setTargetManualRPS(shooter.getSpeedFromDistance(turret.getAimmer().getDistanceTurretToVirtualHub()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
