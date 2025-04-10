// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabber;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Leds.Leds;
import frc.robot.subsystems.Scorer.Scorer;

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
  public final Scorer  scorer = new Scorer();
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();
  private final Vision vision = new Vision(swerveSubsystem);
  private final Leds leds = new Leds();
  boolean slowmode = false;

  // private final Vision Vision = new Vision(swerveSubsystem, null);
public final Autos autos = new Autos(swerveSubsystem, elevator, scorer, climber, algaeGrabber, leds);

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController fightstick = new CommandXboxController(1);
  CommandXboxController sofiabox = new CommandXboxController(2);
  CommandXboxController testController = new CommandXboxController(3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();

    Trigger isReadyToShootAlgae = swerveSubsystem.isWithinShootingRange
    .and(algaeGrabber.isAlgaeInBreakbeam)
    .whileTrue(leds.algae())
    .whileTrue(new StartEndCommand(
      ()->SmartDashboard.putBoolean("barge/Shot OK",true),
      ()->SmartDashboard.putBoolean("barge/Shot OK",false)
    ));

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

    driver.a().toggleOnTrue(swerveSubsystem.driveCommandAllianceManaged(
      ()->-driver.getLeftY()/4.0, 
      ()->-driver.getLeftX()/4.0,
      ()->-driver.getRightX()/4.0
    ));

    // driver.a().whileTrue(
    //   swerveSubsystem.driveCommandAllianceManaged(
    //     ()->-driver.getLeftY(), 
    //     ()->-driver.getLeftX(),
    //     ()->-driver.getRightX()+vision.getRotationDouble()*1
    //     ));

    //swaps tag relative to robot relative
    driver.leftBumper().whileTrue(swerveSubsystem.pathToCoralRight());
    driver.rightBumper().whileTrue(swerveSubsystem.pathToCoralLeft());
    driver.rightTrigger().whileTrue(swerveSubsystem.pathToCoralLeftAuto());
    driver.leftTrigger().whileTrue(swerveSubsystem.pathToCoralRightAuto());
    (driver.povRight().and(driver.povLeft())).whileTrue(swerveSubsystem.pathToReefAlgae(

    ));

  }

  /*********************************
  * OPERATOR BINDINGS
  **********************************/
  
  private void configureOperatorBindings() {

    fightstick.axisLessThan(1, 0)
    .or(sofiabox.button(9))
    .whileTrue(new ParallelCommandGroup(
      climber.prepareToClimb(),
      elevator.moveToPoseUnchecked(elevator.kClimbing),
      algaeGrabber.stop(),
      scorer.runCoralScorer(-2500).withTimeout(0.5)
    ));

    fightstick.axisGreaterThan(1, 0)
      .or(sofiabox.button(3))
      .whileTrue(climber.climb())
      .whileTrue(algaeGrabber.climb())
      ;

    //operator.x().whileTrue(elevator.moveToPoseSafe(elevator.kL1));;
    fightstick.back().or(sofiabox.button(7))
      .whileTrue(elevator.moveToPoseSafe(elevator.kL2Algae).alongWith(scorer.holdAlgae()));
    fightstick.start().or(sofiabox.button(1))
      .whileTrue(elevator.moveToPoseSafe(elevator.kL3Algae).alongWith(scorer.holdAlgae()));
    fightstick.x().or(sofiabox.button(10))
      .whileTrue(elevator.moveToStationPickup().alongWith(scorer.loadCoral()));
    fightstick.y().or(sofiabox.button(4))
      .whileTrue(elevator.moveToPoseSafe(elevator.kL2));
    fightstick.a().or(sofiabox.button(5))
      .whileTrue(elevator.moveToPoseSafe(elevator.kL3));
    fightstick.b().or(sofiabox.button(6))
      .whileTrue(elevator.moveToPoseSafe(elevator.kL4));

    fightstick.rightBumper().or(sofiabox.button(17))
    .whileTrue(goToDefenseMode());

    fightstick.rightTrigger().or(sofiabox.button(11))
    .and(elevator.isAtScorePose).whileTrue(scorer.scoreCoral().repeatedly());//Outake


    fightstick.rightStick().or(sofiabox.button(8))
    .whileTrue(algaeGrabber.shootAlgae())
    .whileTrue(elevator.moveToPoseSafe(elevator.kStowed));


    fightstick.leftBumper().or(sofiabox.button(2))
    .whileTrue(algaeGrabber.intakeFromGround())
    .whileTrue(elevator.moveToPoseSafe(elevator.kStowedUp))
    ;

    fightstick.leftStick().or(sofiabox.button(14))
    .whileTrue(algaeGrabber.scoreProcessor());

    fightstick.leftTrigger().or(sofiabox.button(13))
    .whileTrue(algaeGrabber.algaeUnstuck());

    //cant do, gearing.
    // sofiabox.button(12)
    // .whileTrue(algaeGrabber.)

    //deadline (move to positions, )


    // sofiabox.button(15).whileTrue(
    //   new ParallelCommandGroup(
    //     elevator.moveToPoseSafe(elevator.kShooterIntake),
    //     algaeGrabber.newIntakeFromElevator(),
    //     scorer.holdAlgae()
    //   ).until(elevator.isAtTargetPosition.and(algaeGrabber.isAtTargetAngle))
    //   .andThen(new ParallelDeadlineGroup(
    //     algaeGrabber.newIntakeFromElevator(),
    //     scorer.dropAlgae(),
    //     elevator.holdPosition()
    //     )
    //   //algaegrabber hold intake
    //   //scoreer stop/drop
    //   //elevator hold position

    //   ));
      
    //     ;
  

    // sofiabox.button(15).whileTrue(scorer.loadCoral());


  }

  /**
   * @return the A sequence that runs in Test mode for testing and only testing
   * Because autos go in Autos now.
   */
  public Command getProgrammingTestSequence() {
    return Commands.sequence(
      new ParallelCommandGroup(
      elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
      swerveSubsystem.driveCommandRobotRelative(()->-0.005,()->0.13, ()->0.0)
        .until(scorer.isBranchInRange)
        .withTimeout(2),

      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      autos.scoreAtL4()    
    ));

  }
  
  //Refolding Process
  public Command goToDefenseMode() {
    return Commands.sequence(
      climber.setAngle(()->30.2).withTimeout(3),
      elevator.moveToPoseSafe(elevator.kDefense).until(elevator.isAtTargetAngle).withTimeout(3)
      // elevator.moveToHeightUnfoldHighPrecision(3.78)
    );
  }


  ////////////////////////////////////////////////////////////////////////
  /// PUT AUTOS IN THE AUTOS FILE DON"T PUT THEM HERE WE CAN DO BETTER ///
  /// ////////////////////////////////////////////////////////////////////


  public Command getUnfoldRobot() {
    return Commands.sequence(
      elevator.moveToPoseUnchecked(elevator.kStowedUp).until(elevator.isAtTargetPosition),
      climber.setAngle(()->30.2)
    );
  }
}
