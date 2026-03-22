// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultTurret extends Command {

  Shooter shooter;
  Turret turret;
  CommandXboxController shooterController;

  /** Creates a new DefaultTurret. */
  public DefaultTurret(Shooter s, Turret t, CommandXboxController c) {
    shooter = s;
    turret = t;
    shooterController = c;

    addRequirements(turret);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  //hehehehehehehehehe 67 67 67 sqrt6 sqrt7 = mason was here
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setDegreesReference(turret.getTargetTurretSetpoint() + MathUtil.applyDeadband(shooterController.getRightX(), 0.1) * -1, 0);
    shooter.setTargetManualRPS(shooter.getTargetManualRPS() - MathUtil.applyDeadband(shooterController.getLeftY(), 0.1) * 0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
