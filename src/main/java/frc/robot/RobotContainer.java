// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.RobotDriveCommand;
import frc.robot.commands.ShootCommands.HoodCommand;
import frc.robot.commands.ShootCommands.ShootCommand;
import frc.robot.commands.TurretCommands.AimCommand;
import frc.robot.commands.TurretCommands.StartTurretCommand;
import frc.robot.commands.TurretCommands.TurnTurretCommand;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Joystick driver = new Joystick(0);
  public static Joystick operator = new Joystick(1);


  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();
  TurretSubsystem turretSub = new TurretSubsystem();;
  HopperSubsystem hopperSub = new HopperSubsystem();
  IntakeSubsystem intakeSub = new IntakeSubsystem();
  ShooterSubsystem shooterSub = new ShooterSubsystem();
  HoodSubsystem hoodSub = new HoodSubsystem();
  KickerSubsystem kickSub = new KickerSubsystem();

  PathPlannerTrajectory path;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("Hood Angle input", 14);
    SmartDashboard.putNumber("RPM input", 1800);
    path = PathPlanner.loadPath("Auto Path", 4, 2, true);

    // Configure the button bindings
    configureButtonBindings();

    

    turretSub.setDefaultCommand(new SequentialCommandGroup(
      new TurnTurretCommand(turretSub),
      new StartTurretCommand(turretSub),
      new AimCommand(turretSub)));

    hoodSub.setDefaultCommand(new HoodCommand(hoodSub));


    //passes conditional command into the default command of drive
    driveSub.setDefaultCommand(
      new FieldDriveCommand(
        () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        () -> modifyAxis(driver.getRawAxis(2)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
        driveSub
      ));
        
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driver
    JoystickButton reset = new JoystickButton(driver, 4);
    JoystickButton changeDrive = new JoystickButton(driver,2);
    JoystickButton shoot = new JoystickButton(driver, 8);
    JoystickButton raiseHood = new JoystickButton(driver, 3);
    JoystickButton lowerHood = new JoystickButton(driver, 1);
    JoystickButton intake = new JoystickButton(driver, 7);



    raiseHood.whileActiveContinuous(
      new StartEndCommand(
        ()-> hoodSub.setHood(0.1), 
        ()-> hoodSub.setHood(0),
        hoodSub));

    lowerHood.whileActiveContinuous(
      new StartEndCommand(
        ()-> hoodSub.setHood(-0.1), 
        ()-> hoodSub.setHood(0),
        hoodSub));

    intake.whileActiveContinuous(
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> intakeSub.setIntakeMotor(0.5), 
          ()-> intakeSub.setIntakeMotor(0),
          intakeSub),
        new StartEndCommand(
          ()-> hopperSub.setMotors(.5), 
          ()-> hopperSub.setMotors(0),
          hopperSub)));

    

    shoot.whileActiveContinuous(
      new ParallelCommandGroup(
        new ShootCommand(shooterSub),
        new StartEndCommand(
        ()-> kickSub.setKicker(.8), 
        ()-> kickSub.setKicker(0.0),
        kickSub)));

    reset.whenPressed(new InstantCommand(driveSub::zeroGyroscope, driveSub));

    changeDrive.toggleWhenPressed(
      new RobotDriveCommand(
      () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(2)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
      driveSub
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // driveSub.resetOdometry(Robot.autoTrajectory.getInitialPose());

    var thetaController = new ProfiledPIDController(Constants.ThetaController, .6, .1, Constants.thetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
    path,
    driveSub::getPose,
    Constants.m_kinematics,
    new PIDController(Constants.XController, 0, 0),
    new PIDController(Constants.YController, 0, 0),
    thetaController,
    driveSub::setModules,
    driveSub);

    // Run path following command, then stop at the end.
    return command.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0)));
    // return command;

    // Run path following command, then stop at the end.
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.1) {
        return (value - deadband) / (1.0 - deadband);
      } 
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    // SmartDashboard.putNumber("left x", driver.getLeftX());
    // SmartDashboard.putNumber("left y", driver.getLeftY());
    // SmartDashboard.putNumber("right x", driver.getRawAxis(4));

    return value;
  }
}

// this is where possible scrap code is stored

        // this is possible conditional command code
        // () -> driveSub.robotOrientedDrive(
        //   () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond),
        // () -> driveSub.fieldOrientedDrive(
        //   () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond),
        // true));