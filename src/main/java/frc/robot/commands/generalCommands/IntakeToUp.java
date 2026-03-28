// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.IntakeArm;
import frc.robot.subsystems.swervedrive.IntakeArm.IntakeArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToUp extends InstantCommand {

  IntakeArm intakeArm;
  public IntakeToUp(IntakeArm i) {
    intakeArm = i;
    addRequirements(intakeArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (intakeArm.getIntakeArmState() == IntakeArmState.NONE) {
    //   intakeArm.setReference(Constants.intakeUpSetpoint, 0);
    // } else {
    //   intakeArm.setReference(Constants.intakeUpSetpoint, 0, IntakeArmState.RETRACTED);
    // }
    intakeArm.setReference(Constants.intakeUpSetpoint, 0, IntakeArmState.RETRACTED);
  }
}
