// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generalCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swervedrive.Aimmer.turretControlState;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretSetToCustomSetting extends InstantCommand {


  Turret turret;
  Shooter shooter;
  Pose2d botPose;
  public TurretSetToCustomSetting(Shooter s, Turret t, Pose2d botPose) {
    shooter = s;
    turret = t;
    this.botPose = botPose;

    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.getAimmer().setControlState(turretControlState.FULL_MANUAL);
    turret.setDegreesReference(-turret.getAimmer().getAngleToRealBlueHubFromArbBotPose(botPose).getDegrees(), 0);
    shooter.setTargetManualRPS(shooter.getSpeedFromDistance(turret.getAimmer().getDistanceTurretToRealBlueHubFromArbBotPose(botPose)));

  }
}
