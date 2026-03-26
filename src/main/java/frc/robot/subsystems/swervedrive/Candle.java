// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix6.controls.EmptyAnimation;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Aimmer.turretControlState;

public class Candle extends SubsystemBase {

  CANdle candle = new CANdle(51);
  RainbowAnimation animation = new RainbowAnimation(0,67);
  SolidColor redSolid = new SolidColor(0, 67);
  StrobeAnimation redBlink = new StrobeAnimation(0, 67);
  SolidColor greenSolid = new SolidColor(0, 67);
  StrobeAnimation greenBlink = new StrobeAnimation(0, 67);
  SolidColor yellowSolid = new SolidColor(0, 67);
  StrobeAnimation yellowBlink = new StrobeAnimation(0, 67);
  SolidColor orangeSolid = new SolidColor(0, 67);
  StrobeAnimation orangeBlink = new StrobeAnimation(0, 67);
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
    blinkingGreen,
    yellow,
    orange,
    blinkingYellow,
    blinkingOrange
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
      redBlink.withColor(new RGBWColor(250, 0, 0, 0)).withFrameRate(7);
      greenSolid.withColor(new RGBWColor(0, 250, 0, 0));
      greenBlink.withColor(new RGBWColor(0, 250, 0, 0)).withFrameRate(7);
      yellowSolid.withColor(new RGBWColor(250, 250, 0, 0));
      yellowBlink.withColor(new RGBWColor(250, 250, 0, 0)).withFrameRate(7);
      orangeSolid.withColor(new RGBWColor(250, 165, 0, 0));
      orangeBlink.withColor(new RGBWColor(250, 165, 0, 0)).withFrameRate(7);


  }

  boolean shooterRPSCloseEnough = false;
  boolean turretInBounds = false;
  boolean botTooClose = false;
  boolean turretAndBotInBounds = false;
  turretControlState controlState = turretControlState.FULL_MANUAL;

  @Override
  public void periodic() {

    shooterRPSCloseEnough = shooter.isCloseEnough();
    turretInBounds = aimmer.getInBounds();
    botTooClose = aimmer.getTurretIsTooClose();
    turretAndBotInBounds = (!botTooClose) && turretInBounds;
    controlState = aimmer.getControlState();

    SmartDashboard.putBoolean("LEDSHOOTERRPSCLOSEENOUGH", shooterRPSCloseEnough);
    SmartDashboard.putBoolean("LEDTURRETINBOUNDS", turretInBounds);
    

    switch (controlState) {
      case FULL_AUTO:
        if (shooterRPSCloseEnough && turretAndBotInBounds) {
          state = ledState.blinkingGreen;
        }
        if (shooterRPSCloseEnough && !turretAndBotInBounds) {
          state = ledState.blinkingRed;
        }
        if (!shooterRPSCloseEnough && turretAndBotInBounds) {
          state = ledState.green;
        }
        if (!shooterRPSCloseEnough && !turretAndBotInBounds) {
          state = ledState.red;
        }
      case FULL_MANUAL:
        if (shooterRPSCloseEnough) {
          state = ledState.blinkingOrange;
        } else {
          state = ledState.orange;
        }
      case TURRETAUTO_SHOOTERMANUAL:
        if (shooterRPSCloseEnough && turretAndBotInBounds) {
          state = ledState.blinkingYellow;
        }
        if (shooterRPSCloseEnough && !turretAndBotInBounds) {
          state = ledState.blinkingOrange;
        }
        if (!shooterRPSCloseEnough && turretAndBotInBounds) {
          state = ledState.yellow;
        }
        if (!shooterRPSCloseEnough && !turretAndBotInBounds) {
          state = ledState.orange;
        }
    }
    

    if (prevState != state) {
      candle.setControl(new EmptyAnimation(0));
      if (state == ledState.blinkingGreen) {
        candle.setControl(greenBlink.withSlot(0));
      }
      if (state == ledState.green) {
        candle.setControl(greenSolid);
      }
      if (state == ledState.blinkingRed) {
        candle.setControl(redBlink.withSlot(0));
      }
      if (state == ledState.red) {
        candle.setControl(redSolid);
      }
      if (state == ledState.blinkingOrange) {
        candle.setControl(orangeBlink.withSlot(0));
      }
      if (state == ledState.orange) {
        candle.setControl(orangeSolid);
      }
      if (state == ledState.blinkingYellow) {
        candle.setControl(yellowBlink.withSlot(0));
      }
      if (state == ledState.yellow) {
        candle.setControl(yellowSolid);
      }
      prevState = state;
    }

  }
}
