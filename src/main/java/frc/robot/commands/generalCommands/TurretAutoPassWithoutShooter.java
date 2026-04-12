// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Turret;
import frc.robot.subsystems.swervedrive.Aimmer.turretControlState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAutoPassWithoutShooter extends Command {
  /** Creates a new TurretAutoPass. */

  Turret turret;
  SwerveSubsystem swerve;
  Shooter shooter;
  CommandXboxController controller;
  public TurretAutoPassWithoutShooter(Turret t, SwerveSubsystem s, Shooter ss, CommandXboxController c) {
    turret = t;
    swerve = s;
    shooter = ss;
    controller = c;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.getAimmer().setControlState(turretControlState.TURRETAUTO_SHOOTERMANUAL_PASSING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setDegreesReference(-turret.getAimmer().getAngleToLineOfYourAlliance().getDegrees(), 0);
    shooter.setTargetManualRPS(shooter.getTargetManualRPS() - MathUtil.applyDeadband(controller.getLeftY(), 0.1) * 0.5);
    //shooter.setTargetManualRPS(shooter.getSpeedFromDistance(turret.getAimmer().getDistanceBotToPassingPose()));
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
