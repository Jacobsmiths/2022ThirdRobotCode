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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.KickCommand;
import frc.robot.commands.LowerClimbCommand;
import frc.robot.commands.LowerIntakeCommand;
import frc.robot.commands.LowerHoodCommand;
import frc.robot.commands.RaiseClimbCommand;
import frc.robot.commands.RaiseHoodCommand;
import frc.robot.commands.RaiseIntakeCommand;
import frc.robot.commands.RobotDriveCommand;
import frc.robot.commands.SetHoodCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootPIDCommand;
import frc.robot.commands.StartTurretCommand;
import frc.robot.commands.TurnTurretCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
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
  TurretSubsystem turretSub = new TurretSubsystem();
  HopperSubsystem hopperSub = new HopperSubsystem();
  IntakeSubsystem intakeSub = new IntakeSubsystem();
  ShooterSubsystem shooterSub = new ShooterSubsystem();
  HoodSubsystem hoodSub = new HoodSubsystem();
  LimelightSubsystem limeSub = new LimelightSubsystem();
  ClimbSubsystem climbSub = new ClimbSubsystem();
  KickerSubsystem kickSub = new KickerSubsystem();

  PathPlannerTrajectory path;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    path = PathPlanner.loadPath("Auto Path", 4, 2, true);

    // Configure the button bindings
    configureButtonBindings();

    // turretSub.setDefaultCommand(new SequentialCommandGroup(
    //   new TurnTurretCommand(turretSub),
    //   new StartTurretCommand(turretSub),
    //   new AimCommand(turretSub, limeSub)));

    // shooterSub.setDefaultCommand(new ShootPIDCommand(shooterSub,limeSub));

    // hoodSub.setDefaultCommand(new SetHoodCommand(hoodSub, limeSub));

    //passes conditional command into the default command of drive
    driveSub.setDefaultCommand(new FieldDriveCommand
    (
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
    JoystickButton shoot = new JoystickButton(operator, 8);
    JoystickButton raiseIntake = new JoystickButton(driver, 5);
    JoystickButton lowerIntake = new JoystickButton(driver, 6);


    //OPerator
    JoystickButton raiseHood = new JoystickButton(operator, 1);
    JoystickButton lowerHood = new JoystickButton(operator, 3);
    JoystickButton raiseClimb = new JoystickButton(driver, 8);
    JoystickButton lowerClimb = new JoystickButton(driver, 7);
    JoystickButton intake = new JoystickButton(operator, 2);
    // JoystickButton kicker = new JoystickButton(driver, 8);




    raiseClimb.whileActiveContinuous(new RaiseClimbCommand(climbSub));
    lowerClimb.whileActiveContinuous(new LowerClimbCommand(climbSub));


    raiseHood.whileActiveContinuous(new RaiseHoodCommand(hoodSub));
    lowerHood.whileActiveContinuous(new LowerHoodCommand(hoodSub));
    
    raiseIntake.whileActiveContinuous(new RaiseIntakeCommand(intakeSub));
    lowerIntake.whileActiveContinuous(new LowerIntakeCommand(intakeSub));
    
    shoot.whileActiveContinuous(new ShootCommand(shooterSub));

    // shoot.whileActiveOnce(new ParallelCommandGroup(
    //   new ShootPIDCommand(shooterSub,limeSub),
    //   new KickCommand(kickSub)));
     
    // kicker.whileActiveContinuous(new ParallelCommandGroup 
    // (
    //   new KickCommand(kickSub),
    //   new ShootCommand(shooterSub)
    // ));
    // kicker.whileActiveContinuous(new KickCommand(kickSub));

    intake.whileActiveContinuous(new IntakeCommand(intakeSub, hopperSub));
    reset.whenPressed(new InstantCommand(driveSub::zeroGyroscope, driveSub));

    changeDrive.toggleWhenPressed(new RobotDriveCommand(
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

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //   Robot.autoTrajectory,
    //   driveSub::getPose, // Functional interface to feed supplier
    //   Constants.m_kinematics,
    //   new PIDController(Constants.XController, 0, 0),
    //   new PIDController(Constants.YController, 0, 0),
    //   thetaController,
    //   // (SwerveModuleState[] states) -> driveSub.setModules(states),
    //   driveSub::setModules,
    //   driveSub);

    // // // Reset odometry to the starting pose of the trajectory.
    // driveSub.resetOdometry(Robot.autoTrajectory.getInitialPose());

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
    // return command.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0)));
    return command;

    // Run path following command, then stop at the end.
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.2) {
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