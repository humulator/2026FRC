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
import frc.robot.subsystems.swervedrive.Feeder.feederState;

public class Candle extends SubsystemBase {

  CANdle candle = new CANdle(51);
  RainbowAnimation animation = new RainbowAnimation(0,67);

  SolidColor redSolid = new SolidColor(0, 67);
  StrobeAnimation redBlink = new StrobeAnimation(0, 67);
  StrobeAnimation redFastBlink = new StrobeAnimation(0, 67);

  SolidColor greenSolid = new SolidColor(0, 67);
  StrobeAnimation greenBlink = new StrobeAnimation(0, 67);
  StrobeAnimation greenFastBlink = new StrobeAnimation(0, 67);

  SolidColor yellowSolid = new SolidColor(0, 67);
  StrobeAnimation yellowBlink = new StrobeAnimation(0, 67);
  StrobeAnimation yellowFastBlink = new StrobeAnimation(0, 67);
  
  SolidColor blueSolid = new SolidColor(0, 67);
  StrobeAnimation blueBlink = new StrobeAnimation(0, 67);
  StrobeAnimation blueFastBlink = new StrobeAnimation(0, 67);

  SolidColor whiteSolid = new SolidColor(0, 67);
  StrobeAnimation whiteBlink = new StrobeAnimation(0, 67);
  StrobeAnimation whiteFastBlink = new StrobeAnimation(0, 67);

  SolidColor cyanSolid = new SolidColor(0, 67);
  StrobeAnimation cyanBlink = new StrobeAnimation(0, 67);
  StrobeAnimation cyanFastBlink = new StrobeAnimation(0, 67);
  /** Creates a new Candle. */

  Aimmer aimmer;
  Turret turret;
  Shooter shooter;
  IntakeArm intakeArm;
  IntakeRollers intakeRollers;
  Kickup kickup;
  Feeder feeder;

  enum ledState{
    red,
    green,
    blinkingRed,
    blinkingGreen,
    yellow,
    blue,
    blinkingYellow,
    blinkingBlue,
    fastRed,
    fastGreen,
    fastYellow,
    fastBlue,
    white,
    cyan,
    blinkingWhite,
    blinkingCyan,
    fastWhite,
    fastCyan
  }
  ledState state = ledState.red;
  ledState prevState = ledState.red;


  public Candle(Aimmer aimmer, Turret turret, Shooter shooter, IntakeArm intakeArm, IntakeRollers intakeRollers, Kickup kickup, Feeder feeder) {
      this.aimmer = aimmer;
      this.turret = turret;
      this.shooter = shooter;
      this.intakeArm = intakeArm;
      this.intakeRollers = intakeRollers;
      this.kickup = kickup;
      this.feeder = feeder;

      redSolid.withColor(new RGBWColor(250, 0, 0, 0));
      redBlink.withColor(new RGBWColor(250, 0, 0, 0)).withFrameRate(4);
      redFastBlink.withColor(new RGBWColor(250, 0, 0, 0)).withFrameRate(7);

      greenSolid.withColor(new RGBWColor(0, 250, 0, 0));
      greenBlink.withColor(new RGBWColor(0, 250, 0, 0)).withFrameRate(4);
      greenFastBlink.withColor(new RGBWColor(0, 250, 0, 0)).withFrameRate(7);

      yellowSolid.withColor(new RGBWColor(250, 200, 0, 0));
      yellowBlink.withColor(new RGBWColor(250, 200, 0, 0)).withFrameRate(4);
      yellowFastBlink.withColor(new RGBWColor(250, 200, 0, 0)).withFrameRate(7);

      blueSolid.withColor(new RGBWColor(0, 0, 255, 0));
      blueBlink.withColor(new RGBWColor(0, 0, 255, 0)).withFrameRate(4);
      blueFastBlink.withColor(new RGBWColor(0, 0, 255, 0)).withFrameRate(7);

      whiteSolid.withColor(new RGBWColor(100, 100, 100, 100));
      whiteBlink.withColor(new RGBWColor(100, 100, 100, 100)).withFrameRate(4);
      whiteFastBlink.withColor(new RGBWColor(100, 100, 100, 100)).withFrameRate(7);

      cyanSolid.withColor(new RGBWColor(0, 200, 200, 0));
      cyanBlink.withColor(new RGBWColor(0, 200, 200, 0)).withFrameRate(4);
      cyanFastBlink.withColor(new RGBWColor(0, 200, 200, 0)).withFrameRate(7);

  }

  boolean shooterRPSCloseEnough = false;
  boolean turretInBounds = false;
  boolean botTooClose = false;
  boolean turretAndBotInBounds = false;
  boolean changeToActive = true;
  boolean changeToInactive = true;
  boolean feederOn = false;
  turretControlState controlState = turretControlState.FULL_MANUAL;

  @Override
  public void periodic() {

    shooterRPSCloseEnough = shooter.isCloseEnough();
    turretInBounds = aimmer.getInBounds();
    botTooClose = aimmer.getTurretIsTooClose();
    turretAndBotInBounds = (!botTooClose) && turretInBounds;
    controlState = aimmer.getControlState();
    changeToActive = aimmer.getChangeToActive();
    changeToInactive = aimmer.getChangeToInactive();
    feederOn = feeder.getFeederState() == feederState.ON;

    SmartDashboard.putBoolean("LEDSHOOTERRPSCLOSEENOUGH", shooterRPSCloseEnough);
    SmartDashboard.putBoolean("LEDTURRETINBOUNDS", turretInBounds);
    SmartDashboard.putString("LEDControlState", controlState.toString());
    SmartDashboard.putString("LEDColor", state.toString());
    

    switch (controlState) {
      case FULL_AUTO:
        if (shooterRPSCloseEnough && turretAndBotInBounds) {
          if (feederOn) {
            state = ledState.fastGreen;
          } else {
            state = ledState.blinkingGreen;
          }
        }
        if (shooterRPSCloseEnough && !turretAndBotInBounds) {
          if (feederOn) {
            state = ledState.fastRed;
          } else {
            state = ledState.blinkingRed;
          }
        }
        if (!shooterRPSCloseEnough && turretAndBotInBounds) {
          state = ledState.green;
        }
        if (!shooterRPSCloseEnough && !turretAndBotInBounds) {
          state = ledState.red;
        }
      break;
      case FULL_MANUAL:
        if (shooterRPSCloseEnough) {
          if (feederOn) {
            state = ledState.fastBlue;
          } else {
            state = ledState.blinkingBlue;
          }
        } else {
          state = ledState.blue;
        }
      break;
      case TURRETAUTO_SHOOTERMANUAL:
        if (shooterRPSCloseEnough && turretAndBotInBounds) {
         if (feederOn) {
            state = ledState.fastYellow;
          } else {
            state = ledState.blinkingYellow;
          }
        }
        if (shooterRPSCloseEnough && !turretAndBotInBounds) {
          if (feederOn) {
            state = ledState.fastRed;
          } else {
            state = ledState.blinkingRed;
          }
        }
        if (!shooterRPSCloseEnough && turretAndBotInBounds) {
          state = ledState.yellow;
        }
        if (!shooterRPSCloseEnough && !turretAndBotInBounds) {
          state = ledState.red;
        }
      break;
    }

    if (changeToInactive) {
      if (shooterRPSCloseEnough) {
        if (feederOn) {
            state = ledState.fastWhite;
          } else {
            state = ledState.blinkingWhite;
          }
      } else {
        state = ledState.white;
      }
    }

    if (changeToActive) {
      if (shooterRPSCloseEnough) {
        if (feederOn) {
            state = ledState.fastCyan;
          } else {
            state = ledState.blinkingCyan;
          }
      } else {
        state = ledState.cyan;
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
      if (state == ledState.fastGreen) {
        candle.setControl(greenFastBlink);
      }
      if (state == ledState.blinkingRed) {
        candle.setControl(redBlink.withSlot(0));
      }
      if (state == ledState.red) {
        candle.setControl(redSolid);
      }
      if (state == ledState.fastRed) {
        candle.setControl(redFastBlink);
      }
      if (state == ledState.blinkingBlue) {
        candle.setControl(blueBlink.withSlot(0));
      }
      if (state == ledState.blue) {
        candle.setControl(blueSolid);
      }
      if (state == ledState.fastBlue) {
        candle.setControl(blueFastBlink);
      }
      if (state == ledState.blinkingYellow) {
        candle.setControl(yellowBlink.withSlot(0));
      }
      if (state == ledState.yellow) {
        candle.setControl(yellowSolid);
      }
      if (state == ledState.fastYellow) {
        candle.setControl(yellowFastBlink);
      }
      if (state == ledState.blinkingWhite) {
        candle.setControl(whiteBlink.withSlot(0));
      }
      if (state == ledState.white) {
        candle.setControl(whiteSolid);
      }
      if (state == ledState.fastWhite) {
        candle.setControl(whiteFastBlink);
      }
      if (state == ledState.blinkingCyan) {
        candle.setControl(cyanBlink.withSlot(0));
      }
      if (state == ledState.cyan) {
        candle.setControl(cyanSolid);
      }
      if (state == ledState.fastCyan) {
        candle.setControl(cyanFastBlink);
      }
      
      prevState = state;
    }

  }
}





// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.swervedrive;

// import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
// import com.ctre.phoenix6.controls.EmptyAnimation;
// import com.ctre.phoenix6.controls.FireAnimation;
// import com.ctre.phoenix6.controls.LarsonAnimation;
// import com.ctre.phoenix6.controls.RainbowAnimation;
// import com.ctre.phoenix6.controls.RgbFadeAnimation;
// import com.ctre.phoenix6.controls.SingleFadeAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.controls.TwinkleAnimation;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.LarsonBounceValue;
// import com.ctre.phoenix6.signals.RGBWColor;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.swervedrive.Aimmer.turretControlState;
// import frc.robot.subsystems.swervedrive.Feeder.feederState;

// public class Candle extends SubsystemBase {

//   CANdle candle = new CANdle(51);
//   RainbowAnimation animation = new RainbowAnimation(0,67);
//   SolidColor redSolid = new SolidColor(0, 67);
//   StrobeAnimation redBlink = new StrobeAnimation(0, 67);
//   SolidColor greenSolid = new SolidColor(0, 67);
//   StrobeAnimation greenBlink = new StrobeAnimation(0, 67);
//   SolidColor yellowSolid = new SolidColor(0, 67);
//   StrobeAnimation yellowBlink = new StrobeAnimation(0, 67);
//   SolidColor blueSolid = new SolidColor(0, 67);
//   StrobeAnimation blueBlink = new StrobeAnimation(0, 67);
//   LarsonAnimation whiteLarson = new LarsonAnimation(0, 67);

//   /** Creates a new Candle. */

//   Aimmer aimmer;
//   Turret turret;
//   Shooter shooter;
//   IntakeArm intakeArm;
//   IntakeRollers intakeRollers;
//   Kickup kickup;
//   Feeder feeder;

//   enum ledState{
//     red,
//     green,
//     blinkingRed,
//     blinkingGreen,
//     yellow,
//     blue,
//     blinkingYellow,
//     blinkingBlue
//   }

//   enum ledOverlayState{
//     nothing,
//     vibrating
//   }

//   ledState state = ledState.red;
//   ledState prevState = ledState.red;

//   ledOverlayState overlayState = ledOverlayState.nothing;
//   ledOverlayState prevOverlayState = ledOverlayState.nothing;




//   public Candle(Aimmer aimmer, Turret turret, Shooter shooter, IntakeArm intakeArm, IntakeRollers intakeRollers, Kickup kickup, Feeder feeder) {
//       this.aimmer = aimmer;
//       this.turret = turret;
//       this.shooter = shooter;
//       this.intakeArm = intakeArm;
//       this.intakeRollers = intakeRollers;
//       this.kickup = kickup;
//       this.feeder = feeder;

//       redSolid.withColor(new RGBWColor(250, 0, 0, 0));
//       redBlink.withColor(new RGBWColor(250, 0, 0, 0)).withFrameRate(7);
//       greenSolid.withColor(new RGBWColor(0, 250, 0, 0));
//       greenBlink.withColor(new RGBWColor(0, 250, 0, 0)).withFrameRate(7);
//       yellowSolid.withColor(new RGBWColor(250, 200, 0, 0));
//       yellowBlink.withColor(new RGBWColor(250, 200, 0, 0)).withFrameRate(7);
//       blueSolid.withColor(new RGBWColor(0, 0, 255, 0));
//       blueBlink.withColor(new RGBWColor(0, 0, 255, 0)).withFrameRate(7);
//       whiteLarson.withColor(new RGBWColor(100, 100, 100, 100)).withFrameRate(7);
//       whiteLarson.withBounceMode(LarsonBounceValue.Center).withSize(10).withSlot(1);


//   }

//   boolean shooterRPSCloseEnough = false;
//   boolean turretInBounds = false;
//   boolean botTooClose = false;
//   boolean turretAndBotInBounds = false;
//   boolean hubActive = true;
//   turretControlState controlState = turretControlState.FULL_MANUAL;
//   double frameRate = 7;

//   @Override
//   public void periodic() {

//     shooterRPSCloseEnough = shooter.isCloseEnough();
//     turretInBounds = aimmer.getInBounds();
//     botTooClose = aimmer.getTurretIsTooClose();
//     turretAndBotInBounds = (!botTooClose) && turretInBounds;
//     controlState = aimmer.getControlState();
//     hubActive = aimmer.getHubIsActive();

//     SmartDashboard.putBoolean("LEDSHOOTERRPSCLOSEENOUGH", shooterRPSCloseEnough);
//     SmartDashboard.putBoolean("LEDTURRETINBOUNDS", turretInBounds);
//     SmartDashboard.putString("LEDControlState", controlState.toString());
    

//     switch (controlState) {
//       case FULL_AUTO:
//         if (shooterRPSCloseEnough && turretAndBotInBounds) {
//           state = ledState.blinkingGreen;
//         }
//         if (shooterRPSCloseEnough && !turretAndBotInBounds) {
//           state = ledState.blinkingRed;
//         }
//         if (!shooterRPSCloseEnough && turretAndBotInBounds) {
//           state = ledState.green;
//         }
//         if (!shooterRPSCloseEnough && !turretAndBotInBounds) {
//           state = ledState.red;
//         }
//       break;
//       case FULL_MANUAL:
//         if (shooterRPSCloseEnough) {
//           state = ledState.blinkingBlue;
//         } else {
//           state = ledState.blue;
//         }
//       break;
//       case TURRETAUTO_SHOOTERMANUAL:
//         if (shooterRPSCloseEnough && turretAndBotInBounds) {
//           state = ledState.blinkingYellow;
//         }
//         if (shooterRPSCloseEnough && !turretAndBotInBounds) {
//           state = ledState.blinkingRed;
//         }
//         if (!shooterRPSCloseEnough && turretAndBotInBounds) {
//           state = ledState.yellow;
//         }
//         if (!shooterRPSCloseEnough && !turretAndBotInBounds) {
//           state = ledState.red;
//         }
//       break;
//     }

//     if (feeder.getFeederState() == feederState.ON) {
//       frameRate = 7;
//     } else {
//       frameRate = 7;//4;
//     }
    

//     if (prevState != state) {
//       candle.setControl(new EmptyAnimation(0));
//       if (state == ledState.blinkingGreen) {
//         candle.setControl(greenBlink.withFrameRate(frameRate).withSlot(0));
//       }
//       if (state == ledState.green) {
//         candle.setControl(greenSolid);
//       }
//       if (state == ledState.blinkingRed) {
//         candle.setControl(redBlink.withFrameRate(frameRate).withSlot(0));
//       }
//       if (state == ledState.red) {
//         candle.setControl(redSolid);
//       }
//       if (state == ledState.blinkingBlue) {
//         candle.setControl(blueBlink.withFrameRate(frameRate).withSlot(0));
//       }
//       if (state == ledState.blue) {
//         candle.setControl(blueSolid);
//       }
//       if (state == ledState.blinkingYellow) {
//         candle.setControl(yellowBlink.withFrameRate(frameRate).withSlot(0));
//       }
//       if (state == ledState.yellow) {
//         candle.setControl(yellowSolid);
//       }
//       prevState = state;
//     }


//     if (hubActive) {
//       overlayState = ledOverlayState.nothing;
//     } else {
//       overlayState = ledOverlayState.vibrating;
//     }

//     if (prevOverlayState != overlayState) {
//       if (overlayState == ledOverlayState.nothing) {
//         candle.setControl(new EmptyAnimation(1));
//       }
//       if (overlayState == ledOverlayState.vibrating) {
//         candle.setControl(whiteLarson.withSlot(1));
//       }
//       prevOverlayState = overlayState;
//     }

//   }
// }
