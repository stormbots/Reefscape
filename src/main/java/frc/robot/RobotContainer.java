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

  Swerve swerveSubsystem = new Swerve();
  public final Climber climber = new Climber();
  public final CoralIntake intake = new CoralIntake();

  private final Vision Vision = new Vision(swerveSubsystem, null);

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();

    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
      ()->-driver.getLeftY(),
      ()->-driver.getLeftX(),
      ()->-driver.getRightX()
    ));

  }

  /*********************************
  * DRIVER BINDINGS
  **********************************/
  private void configureDriverBindings() {

    driver.start().onTrue(swerveSubsystem.resetGyro());
    //TODO: Slow mode?
    driver.leftBumper().whileTrue(swerveSubsystem.driveCommand(
      ()->-driver.getLeftY()/4.0, 
      ()->-driver.getLeftX()/4.0,
      ()->-driver.getRightX()/4.0
    ));


    driver.rightTrigger().whileTrue(
      swerveSubsystem.pathToCoralRight()
    );
    driver.leftTrigger().whileTrue(
      swerveSubsystem.pathToCoralLeft()
    );
    driver.x().whileTrue(
      swerveSubsystem.pathToReefAlgae()
    );


    // driverController.a().whileTrue(new RunCommand(()->intake.setAngleSpeed(-30, 100), intake));
    // driverController.b().whileTrue(new RunCommand(()->intake.setAngleSpeed(60, 0), intake));
    
    // driverController.b().whileTrue(climber.climb());
    // driverController.a().whileTrue(climber.prepareToClimb());
    
    // driverController.b().whileFalse(intake.stow());
    // driverController.b().whileTrue(intake.intake());
  }

  /*********************************
  * OPERATOR BINDINGS
  **********************************/
  private void configureOperatorBindings() {

    //TODO: ELEVATOR TO L1 (???)
    //TODO: ELEVATOR TO L2
    //TODO: ELEVATOR TO L3
    //TODO: ELEVATOR TO L4

    //TODO: ELEVATOR TO Reef ALGAE


  }

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
    // return swerveSubsystem.pathToCoralLeft();
    return new InstantCommand();
    // return swerveSubsystem.followPath("1Meter");
  }
}
