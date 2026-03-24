// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase {

  CANdle candle = new CANdle(40);
  RainbowAnimation animation = new RainbowAnimation(0,67);
  SolidColor redSolid = new SolidColor(0, 67);
  StrobeAnimation redBlink = new StrobeAnimation(0, 67);
  SolidColor greenSolid = new SolidColor(0, 67);
  StrobeAnimation greenBlink = new StrobeAnimation(0, 67);
  /** Creates a new Candle. */

  Aimmer aimmer;
  Turret turret;
  Shooter shooter;
  IntakeArm intakeArm;
  IntakeRollers intakeRollers;
  Kickup kickup;

  enum ledState{
    red,
    green,
    blinkingRed,
    blinkingGreen
  }
  ledState state = ledState.red;
  ledState prevState = ledState.red;


  public Candle(Aimmer aimmer, Turret turret, Shooter shooter, IntakeArm intakeArm, IntakeRollers intakeRollers, Kickup kickup) {
      this.aimmer = aimmer;
      this.turret = turret;
      this.shooter = shooter;
      this.intakeArm = intakeArm;
      this.intakeRollers = intakeRollers;
      this.kickup = kickup;

      redSolid.withColor(new RGBWColor(250, 0, 0, 0));
      redBlink.withColor(new RGBWColor(250, 0, 0, 0)).withFrameRate(2);
      greenSolid.withColor(new RGBWColor(0, 250, 0, 0));
      greenBlink.withColor(new RGBWColor(0, 250, 0, 0)).withFrameRate(2);

  }

  boolean shooterRPSCloseEnough = false;
  boolean turretInBounds = false;

  @Override
  public void periodic() {
    shooterRPSCloseEnough = shooter.isCloseEnough();
    turretInBounds = aimmer.getInBounds();

    if (shooterRPSCloseEnough && turretInBounds) {
      state = ledState.blinkingGreen;
    }
    if (shooterRPSCloseEnough && !turretInBounds) {
      state = ledState.blinkingRed;
    }
    if (!shooterRPSCloseEnough && turretInBounds) {
      state = ledState.green;
    }
    if (!shooterRPSCloseEnough && !turretInBounds) {
      state = ledState.red;
    }

    if (prevState != state) {
      if (state == ledState.blinkingGreen) {
        candle.setControl(greenBlink);
      }
      if (state == ledState.green) {
        candle.setControl(greenSolid);
      }
      if (state == ledState.blinkingRed) {
        candle.setControl(redBlink);
      }
      if (state == ledState.red) {
        candle.setControl(redSolid);
      }
    }
    prevState = state;

  }
}
