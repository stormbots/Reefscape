// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
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
  public final Swerve swerveSubsystem = new Swerve();
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final CoralIntake intake = new CoralIntake(elevator.isClear);
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();
  private final Vision vision = new Vision(swerveSubsystem);

  boolean slowmode = false;

  // private final Vision Vision = new Vision(swerveSubsystem, null);
  public final Autos autos = new Autos(swerveSubsystem, elevator, climber, intake, algaeGrabber);

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  CommandXboxController bruh = new CommandXboxController(2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();

    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommandAllianceManaged(
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
    driver.povDown().toggleOnTrue(swerveSubsystem.driveCommandAllianceManaged(
      ()->-driver.getLeftY()/4.0, 
      ()->-driver.getLeftX()/4.0,
      ()->-driver.getRightX()/4.0
    ));

    // driver.rightTrigger().whileTrue(new DeferredCommand(()->swerveSubsystem.pidToPoseCommand(FieldNavigation.getCoralLeft(swerveSubsystem.getPose())), Set.of(swerveSubsystem)));
    driver.a().whileTrue(
      swerveSubsystem.driveCommandAllianceManaged(
        ()->-driver.getLeftY(), 
        ()->-driver.getLeftX(),
        ()->-driver.getRightX()+vision.getRotationDouble()*1
        ));

    driver.leftTrigger().whileTrue(swerveSubsystem.pathToCoralRight());
    driver.rightTrigger().whileTrue(swerveSubsystem.pathToCoralLeft());


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
    operator.back().whileTrue(elevator.moveToPoseSafe(elevator.kL2Algae).alongWith(elevator.runCoralScorer(-2500)));
    operator.start().whileTrue(elevator.moveToPoseSafe(elevator.kL3Algae).alongWith(elevator.runCoralScorer(-2500)));
    operator.x().whileTrue(elevator.moveToStationPickup());
    operator.y().whileTrue(elevator.moveToPoseSafe(elevator.kL2));
    operator.a().whileTrue(elevator.moveToPoseSafe(elevator.kL3));
    operator.b().whileTrue(elevator.moveToPoseSafe(elevator.kL4));

    operator.rightBumper().whileTrue(elevator.moveToIntake(intake.readyToLoad));
    operator.rightBumper().whileTrue(intake.intake(elevator.isCoralInScorer));
    operator.rightBumper().whileFalse(intake.stow(elevator.isClear));

    Command moveTo90 = elevator.moveToAngleTrap(()->90);
    moveTo90.addRequirements(elevator);
    operator.rightTrigger().whileTrue(elevator.runCoralScorer(2500).withTimeout(0.5).andThen(moveTo90));//Outake

    // operator.rightStick().whileTrue(elevator.moveToPoseWithScorer(elevator.kL2Coral));
    // operator.rightStick().whileTrue(elevator.moveToPoseWithScorer(elevator.kL3Coral));

    //is actuaklky shoot
    operator.rightStick().whileTrue(algaeGrabber.scoreProcessor());


    // Expected algae control stuff
    operator.leftBumper().whileTrue(algaeGrabber.intakeAlgaeFromFloor());

    // operator.leftTrigger().whileTrue(algaeGrabber.scoreInNetEzMode());
    // operator.leftTrigger().whileTrue(algaeGrabber.scoreProcessor());
    operator.leftTrigger().whileTrue(algaeGrabber.ejectFromFloor());

    //is also algae score
    operator.leftStick().whileTrue(algaeGrabber.eject());

    //TEST, will not work as does not require elevator subsystem due to intracacieswadkn
    // operator.leftStick().whileTrue(elevator.pidScorerBack());

    // operator.rightTrigger(threshold)
    // operator.rightTrigger().whileTrue(elevator.runCoralScorer(-10));


    // operator.axisGreaterThan(0,0).whileTrue(algaeGrabber.scoreProcessor());
    // operator.a().whileTrue(algaeGrabber.prepareToShoot());

    bruh.a().whileTrue(getUnfoldRobot());
    bruh.b().whileTrue(goToDefenseMode());
  }

  /**
   * @return the A sequence that runs in Test mode for testing and only testing
   * Because autos go in Autos now.
   */
  public Command getProgrammingTestSequence() {
    return new InstantCommand();

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
    // return basicCenterAuto();
    // return swerveSubsystem.followPath("1Meter");
    // return Commands.sequence(
    //   algaeGrabber.prepareToShoot().withTimeout(5),
    //   algaeGrabber.scoreProcessor()  
    // );
  }
  
  //Refolding Process
  public Command goToDefenseMode() {
    return Commands.sequence(
      climber.setAngle(()->30.2),
      elevator.moveToPoseSafe(elevator.new ElevatorPose(3.7, 86.7, 0)).until(elevator.isAtTargetAngle),
      intake.setAngle(()-> 65.0).withTimeout(2),
      elevator.moveToHeightUnfoldHighPrecision(3.78)
    );
  }


  ////////////////////////////////////////////////////////////////////////
  /// PUT AUTOS IN THE AUTOS FILE DON"T PUT THEM HERE WE CAN DO BETTER ///
  /// ////////////////////////////////////////////////////////////////////


  public Command getUnfoldRobot() {
    return Commands.sequence(
      elevator.moveToPoseUnchecked(elevator.kStowedUp).until(elevator.isAtTargetPosition),
      intake.setAngle(()-> 60.0).withTimeout(2),
      climber.setAngle(()->30.2)
    );
  }
}
