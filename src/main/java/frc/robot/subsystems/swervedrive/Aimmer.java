// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Aimmer extends SubsystemBase {

  Optional<Alliance> alliance = DriverStation.getAlliance();

  public Pose2d turretPose = new Pose2d();
  Transform2d robotToTurret = new Transform2d(
  new Translation2d(Inches.of(10), Inches.of(0)), new Rotation2d(0)
);

  Pose2d blueAlliancePose = new Pose2d(
    new Translation2d(Meters.of(4.634), Meters.of(4.029)),
    new Rotation2d()
  );

  Pose2d redAlliancePose = new Pose2d(
    new Translation2d(Meters.of(11.931), Meters.of(4.029)),
    new Rotation2d()
  ); 

  LaserCan distanceSensor;

  SwerveSubsystem swerve;

  InterpolatingDoubleTreeMap speedFromDistance = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap voltageFromDistance = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap timeFromDistance = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap distanceFromSpeed = new InterpolatingDoubleTreeMap();

  ChassisSpeeds robotSpeed;

  boolean isInBounds = false;

   public enum turretControlState{
    FULL_MANUAL,
    TURRETAUTO_SHOOTERMANUAL,
    FULL_AUTO,
    FULL_AUTO_PASSING,
    TURRETAUTO_SHOOTERMANUAL_PASSING
  }

  turretControlState controlState = turretControlState.FULL_MANUAL;

  /** Creates a new pose estimator for the hub. */
  public Aimmer(SwerveSubsystem swerve) {
    this.swerve = swerve;

    //METERS as distanc2
    speedFromDistance.put(6.5, 65.0);
    speedFromDistance.put(5.25, 59.5);
    speedFromDistance.put(4.67, 57.0);
    speedFromDistance.put(4.0, 55.0);
    speedFromDistance.put(3.0, 50.0);
    speedFromDistance.put(2.0, 46.5);

    distanceFromSpeed.put(79.0, 6.5);
    distanceFromSpeed.put(68.0, 5.5);
    distanceFromSpeed.put(60.0, 4.7);
    distanceFromSpeed.put(55.0, 4.0);
    distanceFromSpeed.put(50.0, 3.0);
    distanceFromSpeed.put(46.5, 2.0);
    //speedFromDistance.put(5.0, 15.0);


    voltageFromDistance.put(0.0, 0.0);

    //seconds
    timeFromDistance.put(0.0, 0.0);
    timeFromDistance.put(3.0,0.7 * 1.25);
    timeFromDistance.put(4.0, 0.9* 1.25);
    timeFromDistance.put(5.0, 1.0* 1.25);

    robotSpeed = swerve.getFieldVelocity();

    distanceSensor = new LaserCan(0);
    try {
      distanceSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      distanceSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
      distanceSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

  }

  public void setControlState(turretControlState state) {
    controlState = state;
  }

  public turretControlState getControlState() {
    return controlState;
  }

  public void setTurretPoseFromBotPose(Pose2d botPose) {
    turretPose = botPose.plus(robotToTurret);
  }

  public Pose2d getTurretPoseFromArbBotPose(Pose2d botPose) {
    return botPose.plus(robotToTurret);
  }

  public Rotation2d getAngleToLineOfYourAlliance() {
    Translation2d relativeVector = getPoseOfPointOnLineClosestToBot().getTranslation().minus(turretPose.getTranslation());
    Rotation2d fieldAngle = relativeVector.getAngle();
    return fieldAngle.minus(turretPose.getRotation());
  }

  public Rotation2d getAngleToRealHub() {
    Translation2d relativeVector = getHubPose().getTranslation().minus(turretPose.getTranslation());
    Rotation2d fieldAngle = relativeVector.getAngle();
    return fieldAngle.minus(turretPose.getRotation());
  }

  public Rotation2d getAngleToVirtualHub() {
    Translation2d relativeVector = getPoseOfHubWithFieldSpeeds().getTranslation().minus(turretPose.getTranslation());
    Rotation2d fieldAngle = relativeVector.getAngle();
    return fieldAngle.minus(turretPose.getRotation());
  }

  public Rotation2d getAngleToRealBlueHubFromArbBotPose(Pose2d botpose) {
    Pose2d arbTurretPose = getTurretPoseFromArbBotPose(botpose);
    Translation2d relativeVector = getBlueHubPose().getTranslation().minus(arbTurretPose.getTranslation());
    Rotation2d fieldAngle = relativeVector.getAngle();
    return fieldAngle.minus(arbTurretPose.getRotation());
  }

  public boolean getIsRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
    {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;

  }

  public Pose2d getHubPose() {
    boolean blue = !getIsRedAlliance();
    SmartDashboard.putBoolean("alliancecolorblue", blue);
    if (blue) {
      return blueAlliancePose;
    } else {
      return redAlliancePose;
    }
  }

  public Pose2d getBlueHubPose() {
    return blueAlliancePose;
    
  }

  //2 meters x for the blue
  //15 meters y for the red
  public Pose2d getPoseOfPointOnLineClosestToBot() {
    boolean blue = !getIsRedAlliance();
    SmartDashboard.putBoolean("alliancecolorblue2", blue);
    if (blue) {
      return new Pose2d(new Translation2d(2, swerve.getPose().getY()), new Rotation2d());
    } else {
      return new Pose2d(new Translation2d(15, swerve.getPose().getY()), new Rotation2d());
    }
  }

  /** in meters */
  public double getDistanceTurretToRealHub() {
    return turretPose.getTranslation().getDistance(getHubPose().getTranslation());
  }

  /** in meters */
  public double getDistanceBotToPassingPose() {
    return turretPose.getTranslation().getDistance(getPoseOfPointOnLineClosestToBot().getTranslation());
  }

  /** in meters */
  public double getDistanceTurretToVirtualHub() {
    return turretPose.getTranslation().getDistance(getPoseOfHubWithFieldSpeeds().getTranslation());
  }

  public double getDistanceTurretToRealBlueHubFromArbBotPose(Pose2d botPose) {
    Pose2d arbTurretPose = getTurretPoseFromArbBotPose(botPose);
    return arbTurretPose.getTranslation().getDistance(getBlueHubPose().getTranslation());
  }

  public double getTimeFromDistance(double meters) {
    return timeFromDistance.get(meters);
  }

  public double getVoltageFromDistance(double meters) {
    return voltageFromDistance.get(meters);
  }

  public double getSpeedFromDistance(double meters) {
    return speedFromDistance.get(meters);
  }

  public double getDistanceFromSpeed(double speed) {
    return distanceFromSpeed.get(speed);
  }

  public Pose2d getPoseOfHubWithFieldSpeeds() {
    Translation2d hubpose = getHubPose().getTranslation();
    Translation2d robotMovementInShootingTime = new Translation2d(
      swerve.getFieldVelocity().vxMetersPerSecond * getTimeFromDistance(getDistanceTurretToRealHub()), 
      swerve.getFieldVelocity().vyMetersPerSecond * getTimeFromDistance(getDistanceTurretToRealHub()));
    return new Pose2d(hubpose.minus(robotMovementInShootingTime), new Rotation2d());
  }

  boolean localInBounds = true;
  public double inBounds(double target) {
    localInBounds = true;
    if (target > Constants.maxTurretSetpoint) {
      target = Constants.maxTurretSetpoint;
      localInBounds = false;
    }

    if (target < Constants.minTurretSetpoint) {
      target = Constants.minTurretSetpoint;
      localInBounds = false;
    }
    isInBounds = localInBounds;

    return target;
  }

  public boolean getInBounds() {
    return isInBounds;
  }

  public boolean getTurretIsTooClose() {
    return getDistanceTurretToVirtualHub() < Constants.turretTooCloseMeters;
  }

  public double getmmSensor() {
    if (distanceSensor.getMeasurement() == null) {} else {
      return distanceSensor.getMeasurement().distance_mm;
    }
    return 1000000000;
  }

  public boolean getRedAutoWon() {
    String data = DriverStation.getGameSpecificMessage();

    if (data != null) {
      if (!data.equals("")) {
        if (data.equals("R")) {
          return true;
        } else {
          return false;
        }
      }
    }

    return true;
  }

  public boolean WonAuto() {
    boolean redWin = getRedAutoWon();
    boolean redAlliance = getIsRedAlliance();

    if (redWin == redAlliance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getHubIsActive(double offset) {
    boolean autoWin = WonAuto();
    double timeUntil0 = Timer.getMatchTime() - offset;
    if (timeUntil0 < 30) {
      return true;
    }
    if (autoWin) {
      if (timeUntil0 < 55) {
        return true;
      } else if (timeUntil0 < 80) {
        return false;
      } else if (timeUntil0 < 105) {
        return true;
      } else if (timeUntil0 < 130) {
        return false;
      } else {
        return true;
      }
    } else {
      if (timeUntil0 < 55) {
        return false;
      } else if (timeUntil0 < 80) {
        return true;
      } else if (timeUntil0 < 105) {
        return false;
      } else if (timeUntil0 < 130) {
        return true;
      } else {
        return true;
      }
    }

  }

  public double getTimeUntilNextChange(double offset) {
    boolean autoWin = WonAuto();
    double timeUntil0 = Timer.getMatchTime() - offset;
    if (timeUntil0 < 30) {
      return timeUntil0;
    }
    if (autoWin) {
      if (timeUntil0 < 55) {
        return timeUntil0;
      } else if (timeUntil0 < 80) {
        return timeUntil0 - 55;
      } else if (timeUntil0 < 105) {
        return timeUntil0 - 80;
      } else if (timeUntil0 < 130) {
        return timeUntil0 - 105;
      } else {
        return timeUntil0 - 130;
      }
    } else {
      if (timeUntil0 < 55) {
        return timeUntil0 - 30;
      } else if (timeUntil0 < 80) {
        return timeUntil0 - 55;
      } else if (timeUntil0 < 105) {
        return timeUntil0 - 80;
      } else if (timeUntil0 < 130) {
        return timeUntil0 - 105;
      } else {
        return timeUntil0 - 105;
      }
    }

  }

  public boolean getChangeToActive() {
    if (getHubIsActive(0)){
      return false;
    } else if (getHubIsActive(Constants.LEDWarningSeconds)){
      return true;
    } else{
      return false;
    }
  }

  public boolean getChangeToInactive() {
    if (!getHubIsActive(0)){
      return false;
    } else if (!getHubIsActive(Constants.LEDWarningSeconds)){
      return true;
    } else{
      return false;
    }
  }


  @Override
  public void periodic() {
    setTurretPoseFromBotPose(swerve.getPose());
    swerve.getField().getObject("turret").setPose(turretPose);

    SmartDashboard.putNumber("turretTargetCalculated", getAngleToRealHub().getDegrees());
    SmartDashboard.putNumber("distancetohubfromturret", getDistanceTurretToRealHub());
    SmartDashboard.putNumber("number", getSpeedFromDistance(3));
    swerve.getField().getObject("hubButItsActuallyWHereTheBotWantsToShoot").setPose(getPoseOfHubWithFieldSpeeds());
    swerve.getField().getObject("linething").setPose(getPoseOfPointOnLineClosestToBot());
    SmartDashboard.putNumber("DistanceSensorOutput", getmmSensor());
    //SmartDashboard.putNumber("fmstimer", Timer.getMatchTime());
    SmartDashboard.putBoolean("HUBSTATE", getHubIsActive(0));
    SmartDashboard.putNumber("TimeUntilNextShift", Math.round(getTimeUntilNextChange(0) * 10.0) / 10.0);

    // This method will be called once per scheduler run
  }
}
