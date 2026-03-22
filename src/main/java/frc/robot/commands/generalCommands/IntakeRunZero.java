// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.IntakeRollers;
import frc.robot.subsystems.swervedrive.IntakeArm.IntakeArmState;
import frc.robot.subsystems.swervedrive.IntakeRollers.IntakeRollerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRunZero extends InstantCommand {

  IntakeRollers intakeRollers;
  /** Creates a new intakeRunZero. */
  public IntakeRunZero(IntakeRollers i) {
    intakeRollers = i;
    addRequirements(intakeRollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    intakeRollers.setVoltageOpenLoop(0);
  }
}
