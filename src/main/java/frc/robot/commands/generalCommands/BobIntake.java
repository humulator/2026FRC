// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.IntakeArm;
import frc.robot.subsystems.swervedrive.IntakeArm.IntakeArmState;
import frc.robot.subsystems.swervedrive.IntakeRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BobIntake extends Command {

  IntakeArm intakeArm;
  IntakeRollers intakeRollers;

  int ticker = 0;
  /** Creates a new BobIntake. */
  public BobIntake(IntakeArm i, IntakeRollers ii) {

    intakeArm = i;
    intakeRollers = ii;
    addRequirements(intakeArm, intakeRollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeRollers.setVoltageOpenLoop(Constants.IntakeForwardSpeed);
    intakeArm.setReference(Constants.intakeDownSetpoint, 0, IntakeArmState.EXTENDED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ticker >= Constants.intakeTickerNumber) {
      if (intakeArm.getIntakeState() == IntakeArmState.EXTENDED) {
        intakeArm.setReference(Constants.intakeBobSetpoint, 0, IntakeArmState.BOB);
      } else {
        intakeArm.setReference(Constants.intakeDownSetpoint, 0, IntakeArmState.EXTENDED);
      }
      ticker = 0;
    } else {
      ticker++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeRollers.setVoltageOpenLoop(0);
    intakeArm.setReference(Constants.intakeDownSetpoint, 0, IntakeArmState.EXTENDED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
