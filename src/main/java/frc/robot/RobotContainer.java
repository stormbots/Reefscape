// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CoralIntake.CoralIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based i]
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  // public AHRS navxGyro = new AHRS(NavXComType.kMXP_SPI);

  Swerve swerveSubsystem = new Swerve();
  public final Climber climber = new Climber();
  public final CoralIntake intake = new CoralIntake();

  private final Vision Vision = new Vision(swerveSubsystem, null);

  CommandXboxController driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    driverController.start().onTrue(swerveSubsystem.resetGyro());
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
      ()->-driverController.getLeftY(),
      ()->-driverController.getLeftX(),
      ()->-driverController.getRightX()
    ));
    //TODO: Slow mode?
    driverController.leftBumper().whileTrue(swerveSubsystem.driveCommand(
      ()->-driverController.getLeftY()/4.0, 
      ()->-driverController.getLeftX()/4.0,
      ()->-driverController.getRightX()/4.0
    ));


    driverController.rightTrigger().whileTrue(
      swerveSubsystem.pathToCoralRight()
    );
    driverController.leftTrigger().whileTrue(
      swerveSubsystem.pathToCoralLeft()
    );
    driverController.x().whileTrue(
      swerveSubsystem.pathToReefAlgae()
    );


    // driverController.a().whileTrue(new RunCommand(()->intake.setAngleSpeed(-30, 100), intake));
    // driverController.b().whileTrue(new RunCommand(()->intake.setAngleSpeed(60, 0), intake));
    
    // driverController.b().whileTrue(climber.climb());
    // driverController.a().whileTrue(climber.prepareToClimb());
    
    // driverController.b().whileFalse(intake.stow());
    // driverController.b().whileTrue(intake.intake());


    
  }

  // private void configureDefaultCommands(){
  //   climber.setDefaultCommand(climber.setAngle(climber.getPosition()));
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(exampleSubsystem);
    // return new SequentialCommandGroup(
    //   climber.prepareToClimb(),
    //   new WaitCommand(1),
    //   climber.climb(),
    //   new WaitCommand(3),
    //   climber.setAngle(()->45),
    //   // new WaitCommand(1),
    //   climber.stow()
      
    // );
    return swerveSubsystem.pathToCoralLeft();
    // return new InstantCommand();
    // return swerveSubsystem.followPath("1Meter");
  }
}
