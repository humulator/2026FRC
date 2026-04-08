// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Feeder;
import frc.robot.subsystems.swervedrive.Feeder.feederState;
import frc.robot.subsystems.swervedrive.Kickup;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WhileHeldReverseKickupAndFeeder extends Command {

  Kickup kickup;
  Feeder feeder;

  /** Creates a new whileHeldReverseKickupAndFeeder. */
  public WhileHeldReverseKickupAndFeeder(Kickup k, Feeder f) {
    kickup = k;
    feeder = f;
    addRequirements(kickup, feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kickup.setVoltageOpenLoop(-6);
    feeder.setVoltageOpenLoop(-6, feederState.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kickup.setVoltageOpenLoop(0);
    feeder.setVoltageOpenLoop(0, feederState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
