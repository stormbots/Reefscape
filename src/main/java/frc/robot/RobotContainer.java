// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabber;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based i]
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  Swerve swerveSubsystem = new Swerve();
  public final Climber climber = new Climber();
  public final CoralIntake intake = new CoralIntake();
  public final Elevator elevator = new Elevator();
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();

  boolean slowmode = false;

  // private final Vision Vision = new Vision(swerveSubsystem, null);

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
    driver.povDown().toggleOnTrue(swerveSubsystem.driveCommand(
      ()->-driver.getLeftY()/4.0, 
      ()->-driver.getLeftX()/4.0,
      ()->-driver.getRightX()/4.0
    ));


    // driver.rightTrigger().whileTrue(
    //   swerveSubsystem.pathToCoralRight()
    // );
    // driver.leftTrigger().whileTrue(
    //   swerveSubsystem.pathToCoralLeft()
    // );
    /*driver.x().whileTrue(
      swerveSubsystem.pathToReefAlgae()
    );*/


    // driver.a().whileTrue(new RunCommand(()->intake.setAngleSpeed(-30, 100), intake));
    // driver.b().whileTrue(new RunCommand(()->intake.setAngleSpeed(60, 0), intake));
    
    // driver.b().whileTrue(climber.climb());
    // driver.a().whileTrue(climber.prepareToClimb());
    
    // driver.b().whileFalse(intake.stow());
    // driver.b().whileTrue(intake.intake());
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

    //TODO: Make all these bindings associate to operator.

    // driver.a().whileTrue(
    //   elevator.moveToPoseSafe(elevator.kL3)
    // );

    
    // driver.y().whileTrue(
    //   elevator.moveToPoseSafe(elevator.kStowedUp)
    // );

    // //Move to prep and complete motion only if intake is down
    // driver.b().whileTrue(elevator.moveToIntake(intake.readyToLoad));


    // driver.a().whileTrue(
    //   elevator.moveToPoseSafe(elevator.kL3)
    // );

    // driver.x().whileTrue( new ParallelCommandGroup(
    //   climber.prepareToClimb(), elevator.moveToPoseSafe(elevator.kClimbing), intake.stow())
    // );

    // // driver.back().whileTrue(elevator.homeElevator());

    // driver.start().whileTrue(climber.climb());

    // driver.leftTrigger().onTrue(intake.setAngle(()-> -20));
    // driver.rightTrigger().onTrue(intake.setAngle(()-> 90));

    operator.axisLessThan(1, 0).whileTrue(new ParallelCommandGroup(
      climber.prepareToClimb(),
      elevator.moveToPoseUnchecked(elevator.kClimbing),
      intake.setAngle(()->65),
      algaeGrabber.stop()
    ));

    operator.axisGreaterThan(1, 0).whileTrue(climber.climb());

    //operator.x().whileTrue(elevator.moveToPoseSafe(elevator.kL1));
    operator.y().whileTrue(elevator.moveToPoseSafe(elevator.kL2));
    operator.a().whileTrue(elevator.moveToPoseSafe(elevator.kL3));
    operator.b().whileTrue(elevator.moveToPoseSafe(elevator.kL4));

    operator.rightBumper().whileTrue(elevator.moveToIntake(intake.readyToLoad));
    operator.rightBumper().whileTrue(intake.intake());
    operator.rightBumper().whileFalse(intake.stow(elevator.isClear));

    // Expected algae control stuff
    operator.leftBumper().whileTrue(algaeGrabber.intakeAlgaeFromFloor());

    // operator.leftTrigger().whileTrue(algaeGrabber.scoreInNetEzMode());
    operator.leftTrigger().whileTrue(algaeGrabber.scoreProcessor());

    // operator.rightTrigger(threshold)
    // operator.rightTrigger().whileTrue(elevator.runCoralScorer(-10));


    operator.axisGreaterThan(0,0).whileTrue(algaeGrabber.scoreProcessor());
    // operator.a().whileTrue(algaeGrabber.prepareToShoot());
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
    return centerAuto();
    // return new InstantCommand();
    // return swerveSubsystem.followPath("1Meter");
    // return Commands.sequence(
    //   algaeGrabber.prepareToShoot().withTimeout(5),
    //   algaeGrabber.scoreProcessor()  
    // );
  }
  
  public Command getUnfoldRobot() {
    return Commands.sequence(
      elevator.moveToPoseUnchecked(elevator.kStowedUp).until(elevator.isAtTargetPosition),
      intake.setAngle(()-> 60.0).withTimeout(2),
      climber.setAngle(()->30.2)
    );
  }
  //Refolding Process
  public Command getFoldRobot() {
    return Commands.sequence(
      climber.setAngle(()->30.2),
      elevator.moveToPoseSafe(elevator.new ElevatorPose(3.7, 86.7, 0)).until(elevator.isAtTargetAngle),
      intake.setAngle(()-> 65.0).withTimeout(2),
      elevator.moveToHeightUnfoldHighPrecision(3.78)
    );
  }


  ///////////////////////////////////////////////////////////
  /// AUTOS BELOW THIS LINE SORRY IT'S GROSS I HATE IT TOO -DAN
  /// //////////////////////////////////////////////////////////

  public Command centerAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    //Initial Position for Auto on Blue
    swerveSubsystem.resetOdometry(new Pose2d(10.0, 4.05, new Rotation2d(0.0)));
    //Initial Position for Auto on Red
    if(alliance == Alliance.Red){
      swerveSubsystem.resetOdometry(new Pose2d(7.6, 4.05, new Rotation2d(Degrees.of(180))));
    }
    return Commands.sequence(
    swerveSubsystem.pathToCoralLeft(),
    // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
    elevator.moveToPoseSafe(elevator.kL4).until(elevator.isAtTargetPosition),
    elevator.runCoralScorer(2500).withTimeout(1)
    );
  }

}
