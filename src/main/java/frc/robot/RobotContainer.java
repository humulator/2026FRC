// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.generalCommands.BobIntake;
import frc.robot.commands.generalCommands.CalibrateTurret;
import frc.robot.commands.generalCommands.DefaultTurret;
import frc.robot.commands.generalCommands.FeederWhileHeld;
import frc.robot.commands.generalCommands.IntakeRunBackward;
import frc.robot.commands.generalCommands.KickupWhileHeld;
import frc.robot.commands.generalCommands.PrepareShooterThenFeed;
import frc.robot.commands.generalCommands.ToggleIntakeUpDown;
import frc.robot.commands.generalCommands.TurretAuto;
import frc.robot.commands.generalCommands.TurretAutoPass;
import frc.robot.commands.generalCommands.TurretAutoPassWithoutShooter;
import frc.robot.commands.generalCommands.TurretAutoWithoutShooter;
import frc.robot.commands.generalCommands.WhileHeldReverseKickupAndFeeder;
import frc.robot.commands.generalCommands.WhileHeldShootAndFeed;
import frc.robot.commands.generalCommands.WhileHeldShooterOnly;
import frc.robot.commands.generalCommands.IntakeRunForward;
import frc.robot.commands.generalCommands.IntakeRunZero;
import frc.robot.commands.generalCommands.IntakeToDown;
import frc.robot.commands.generalCommands.IntakeToUp;
import frc.robot.subsystems.swervedrive.Aimmer;
import frc.robot.subsystems.swervedrive.Candle;
import frc.robot.subsystems.swervedrive.EdittedController;
import frc.robot.subsystems.swervedrive.Feeder;
import frc.robot.subsystems.swervedrive.IntakeArm;
import frc.robot.subsystems.swervedrive.IntakeRollers;
import frc.robot.subsystems.swervedrive.Kickup;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem.swervePosition;
import frc.robot.subsystems.swervedrive.Turret;
import frc.robot.subsystems.swervedrive.EdittedController.driveSpeedState;
import frc.robot.subsystems.swervedrive.Feeder.feederState;
import frc.robot.subsystems.swervedrive.IntakeArm.IntakeArmState;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import java.util.Set;

import swervelib.SwerveInputStream;
import swervelib.parser.json.modules.DriveConversionFactorsJson;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         EdittedController driverXbox = new EdittedController(0);
  CommandXboxController shooterController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/kraken"));
                                                                                
  Aimmer aimmer = new Aimmer(drivebase);
  Turret turret = new Turret(drivebase, aimmer);
  Shooter shooter = new Shooter(aimmer);
  IntakeArm intakeArm = new IntakeArm();
  IntakeRollers intakeRollers = new IntakeRollers();
  Kickup kickup = new Kickup();
  Feeder feeder = new Feeder();
  Candle candle = new Candle(aimmer, turret, shooter, intakeArm, intakeRollers, kickup, feeder);

  


  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getY() * -1,
                                                                () -> driverXbox.getX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getInvertedRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox.controller::getRightX,
                                                                                             driverXbox.controller::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getY(),
                                                                        () -> -driverXbox.getX())
                                                                    .withControllerRotationAxis(() -> driverXbox.controller.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.controller.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.controller.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.controller.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // driverXbox.controller.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.controller.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                                () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    //  driverXbox.controller.b().whileTrue(
    //      drivebase.driveToPose(
    //          new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                          );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.controller.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.controller.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.controller.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.controller.leftBumper().onTrue(Commands.none());
      driverXbox.controller.rightBumper().onTrue(Commands.none());
    } else
    {
      //driverXbox.controller.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.controller.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.controller.start().whileTrue(Commands.none());
      driverXbox.controller.back().whileTrue(Commands.none());
      driverXbox.controller.leftStick().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.controller.rightBumper().onTrue(Commands.none());
    }

    //H WUAHFUYWGAGAHDSEWRTYUIJKLMNBVGFDRTYUIJKLMNBVGFTYUIOJKMNBHHJYUIJKNBHGYHUJKNBHGUJ

    NamedCommands.registerCommand("TurretAuto", new TurretAuto(turret, drivebase, shooter));
    NamedCommands.registerCommand("WhileHeldShooterOnly", new WhileHeldShooterOnly(shooter, turret, shooterController));
    //NamedCommands.registerCommand("WhileHeldShootAndFeed", new WhileHeldShootAndFeed(new KickupWhileHeld(kickup), new WhileHeldShooterOnly(shooter, turret, shooterController), new FeederWhileHeld(feeder)));
    NamedCommands.registerCommand("WhileHeldShootAndFeed", new SequentialCommandGroup((new WhileHeldShooterOnly(shooter, turret, shooterController).withTimeout(1)), new WhileHeldShootAndFeed(new KickupWhileHeld(kickup), new WhileHeldShooterOnly(shooter, turret, shooterController), new FeederWhileHeld(feeder))));
    // NamedCommands.registerCommand("WhileHeldShootAndFeed", new PrepareShooterThenFeed(
    //   new WhileHeldShooterOnly(shooter, turret, shooterController), 
    //   new WhileHeldShootAndFeed(new KickupWhileHeld(kickup), new WhileHeldShooterOnly(shooter, turret, shooterController), new FeederWhileHeld(feeder))
    //   , shooter));
    NamedCommands.registerCommand("IntakeToDown", new IntakeToDown(intakeArm));
    NamedCommands.registerCommand("IntakeRunForward", new IntakeRunForward(intakeRollers));
    NamedCommands.registerCommand("IntakeToUp", new IntakeToUp(intakeArm));
    //NamedCommands.registerCommand("IntakeToUp", Commands.none());
    NamedCommands.registerCommand("IntakeRunBackward", new IntakeRunBackward(intakeRollers));
    NamedCommands.registerCommand("IntakeRunZero", new IntakeRunZero(intakeRollers));
    NamedCommands.registerCommand("CalibrateTurret", new CalibrateTurret(aimmer, turret));


    Trigger R1WithoutR2 = shooterController.rightBumper().and(shooterController.rightTrigger().negate());
    Trigger R2WithoutR1 = shooterController.rightTrigger();

    Trigger L1WithoutL2 = driverXbox.controller.leftTrigger().and(driverXbox.controller.rightTrigger().negate());
    Trigger L2WithoutL1 = driverXbox.controller.rightTrigger().and(driverXbox.controller.leftTrigger().negate());
    Trigger L2AndL1 = driverXbox.controller.leftTrigger().and(driverXbox.controller.rightTrigger());
    Trigger intakeReverse = driverXbox.controller.back();

    //Intake stuff
    // L1WithoutL2.onTrue(new ParallelCommandGroup(new IntakeToDown(intakeArm)));
    // L1WithoutL2.onFalse(new ParallelCommandGroup(new IntakeToUp(intakeArm)));

    L1WithoutL2.onTrue(new ToggleIntakeUpDown(intakeArm));

    L2WithoutL1.onTrue(new ParallelCommandGroup(new IntakeRunForward(intakeRollers)));
    L2WithoutL1.onFalse(new ParallelCommandGroup(new IntakeRunZero(intakeRollers)));

    L2AndL1.whileTrue(new BobIntake(intakeArm, intakeRollers));

    intakeReverse.onTrue(new IntakeRunBackward(intakeRollers));
    intakeReverse.onFalse(new IntakeRunZero(intakeRollers));    

    // Turret and non-required shooting yUP xLEFT aDOWN bRIGHT
    turret.setDefaultCommand(new DefaultTurret(shooter, turret, shooterController));
    shooterController.y().toggleOnTrue(new TurretAuto(turret, drivebase, shooter));
    shooterController.x().toggleOnTrue(new TurretAutoPass(turret, drivebase, shooter));
    shooterController.a().toggleOnTrue(new TurretAutoWithoutShooter(turret, drivebase, shooter, shooterController));
    shooterController.b().toggleOnTrue(new TurretAutoPassWithoutShooter(turret, drivebase, shooter, shooterController));

    // For the SOTM
    Trigger feederOn = new Trigger(() -> (feeder.getFeederState() == feederState.ON));

    feederOn.onTrue(new InstantCommand(() -> driverXbox.setFactor(0.4, driveSpeedState.slow)));
    feederOn.onFalse(new InstantCommand(() -> driverXbox.setFactor(1, driveSpeedState.normal)));

    // required shooting and feeding
    R1WithoutR2.whileTrue(new WhileHeldShooterOnly(shooter, turret, shooterController));
    shooterController.start().whileTrue(new WhileHeldReverseKickupAndFeeder(kickup, feeder));
    R2WithoutR1.whileTrue(new WhileHeldShootAndFeed(new KickupWhileHeld(kickup), new WhileHeldShooterOnly(shooter, turret, shooterController), new FeederWhileHeld(feeder)));
    
    

    //gyro reset
    driverXbox.controller.x().onTrue(new InstantCommand(() -> driverXbox.setOffset(drivebase.getPose().getRotation().getDegrees() + 180)));
    driverXbox.controller.b().onTrue(new InstantCommand(() -> driverXbox.setOffset(0)));

    Trigger intakeArmNearDown = new Trigger(() -> intakeArm.isNearTarget(Constants.intakeDownSetpoint));
    intakeArmNearDown.onTrue(Commands.runEnd(
        () -> driverXbox.controller.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        () -> driverXbox.controller.getHID().setRumble(RumbleType.kBothRumble, 0)
    ).withTimeout(0.5));

    Trigger intakeArmNearUp = new Trigger(() -> intakeArm.isNearTarget(Constants.intakeUpSetpoint));
    intakeArmNearUp.onTrue(Commands.runEnd(
        () -> driverXbox.controller.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        () -> driverXbox.controller.getHID().setRumble(RumbleType.kBothRumble, 0)
    ).withTimeout(0.5));

    Trigger shooterIsNear = new Trigger(() -> shooter.isCloseEnough());
    shooterIsNear.onTrue(Commands.runEnd(
        () -> shooterController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        () -> shooterController.getHID().setRumble(RumbleType.kBothRumble, 0)
    ).withTimeout(0.5));

    Trigger slowDriveTrain = new Trigger(() -> (driverXbox.getSpeedState() == driveSpeedState.slow));
    slowDriveTrain.whileTrue(Commands.runEnd(
        () -> driverXbox.controller.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        () -> driverXbox.controller.getHID().setRumble(RumbleType.kBothRumble, 0)
    ));



    //Auto Align 

    driverXbox.controller.povLeft().whileTrue(new ConditionalCommand(
        drivebase.driveToPose(new Pose2d(13.537, 0.724, new Rotation2d(Degrees.of(90)))), 
        drivebase.driveToPose(new Pose2d(3.593, 7.423, new Rotation2d(Degrees.of(-90)))), 
        () -> aimmer.getIsRedAlliance()
    ));

    driverXbox.controller.povRight().whileTrue(new ConditionalCommand(
        drivebase.driveToPose(new Pose2d(13.242, 7.281, new Rotation2d(Degrees.of(-112)))), 
        drivebase.driveToPose(new Pose2d(3.429, 0.636, new Rotation2d(Degrees.of(90)))), 
        () -> aimmer.getIsRedAlliance()
    ));

    driverXbox.controller.leftBumper().whileTrue(Commands.defer(() -> drivebase.getUnderTrenchLeftCommand(), Set.of(drivebase)));
    driverXbox.controller.rightBumper().whileTrue(Commands.defer(() -> drivebase.getUnderTrenchRightCommand(), Set.of(drivebase)));

    // driverXbox.controller.rightBumper().whileTrue(drivebase.getUnderTrenchRightCommand());
    // driverXbox.controller.leftBumper().whileTrue(drivebase.getUnderTrenchLeftCommand());
      // // RED 
      // driverXbox.controller.leftTrigger().whileTrue(drivebase.driveToPose(new Pose2d(13.537, 0.724, new Rotation2d(Degrees.of(90))))); // RED LEFT
      // driverXbox.controller.rightTrigger().whileTrue(drivebase.driveToPose(new Pose2d(13.242, 7.281, new Rotation2d(Degrees.of(-112))))); // RED RIGHT

      // // BLUE
      // driverXbox.controller.leftTrigger().whileTrue(drivebase.driveToPose(new Pose2d(3.593, 7.423, new Rotation2d(Degrees.of(-90))))); // BLUE LEFT
      // driverXbox.controller.rightTrigger().whileTrue(drivebase.driveToPose(new Pose2d(3.429, 0.636, new Rotation2d(Degrees.of(90))))); // BLUE RIGHT
    //driverXbox.controller.a().whileTrue(drivebase.driveToPose(new Pose2d(drivebase.getPose().getTranslation(), aimmer.getAngleToVirtualHub().plus(aimmer.turretPose.getRotation()))));

    //shooterController.povUp().whileTrue(new CalibrateTurret(aimmer, turret));

    shooterController.povUp().onTrue(new InstantCommand(() -> shooter.setTargetManualRPS(shooter.getTargetManualRPS() + 3)));
    shooterController.povDown().onTrue(new InstantCommand(() -> shooter.setTargetManualRPS(shooter.getTargetManualRPS() - 3)));

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
