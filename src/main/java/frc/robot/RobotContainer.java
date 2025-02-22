// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabber;
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
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();

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
  
  //Axis 1 (-) = climb
  //Axis 1 (+) = climber in
  //Axis 0 (-) = climber stow
  //disable climber till last 30s?
  //Axis 0 (+) = processor
  //button 5 (LB) = algae intake
  //Axis 2 (LT) = net (dosen't say whether shoot or place)
  //Button 3 (X) = L1
  //Button 4 (Y) = L2
  //Button 1 (A) = L3
  //Button 2 (B) = L4
  //Button 6 (RB) = intake
  //Axis 3 (RT) = score coral
  //Button 7 (share) = rezero (vague, button changed from home as home is not usable)
  //Button 8 (options) = manual mode (bad idea)
  //Button 9 (SL) = eject algae
  //Button 10 (SR) = eject coral



  private void configureOperatorBindings() {

    //TODO: ELEVATOR TO L1 (???)
    //TODO: ELEVATOR TO L2
    //TODO: ELEVATOR TO L3
    //TODO: ELEVATOR TO L4

    //TODO: ELEVATOR TO Reef ALGAE


    // Expected algae control stuff
    operator.rightBumper().whileTrue(intake.intake());
    operator.rightBumper().whileFalse(intake.stow());

    operator.leftBumper().whileTrue(algaeGrabber.intakeAlgaeFromFloor());

    operator.leftTrigger().whileTrue(algaeGrabber.scoreInNetEzMode());

    operator.axisGreaterThan(0,0).whileTrue(algaeGrabber.scoreProcessor());
    // operator.a().whileTrue(algaeGrabber.prepareToShoot());
    // driverController.b().whileTrue(algaeGrabber.scoreInNetEzMode());
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
    // return new InstantCommand();
    // return swerveSubsystem.followPath("1Meter");
    return Commands.sequence(
      algaeGrabber.prepareToShoot().withTimeout(5),
      algaeGrabber.scoreProcessor()  

    );
  }
}
