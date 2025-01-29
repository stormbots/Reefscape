// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private Elevator elevator;
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  Swerve swerveSubsystem = new Swerve();
  CommandXboxController driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.currentMode == Mode.real) {
      elevator = new Elevator("elevator", new ElevatorIOReal());
    } else {
      elevator = new Elevator("elevator", new ElevatorIOSim());
    }
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

    swerveSubsystem.setDefaultCommand(
        swerveSubsystem.driveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(exampleSubsystem);
    // return elevator.runElevator(6.0).withTimeout(10.0);
    // return new RunCommand(() -> elevator.setHeightMeters(2.5), elevator);
    // return swerveSubsystem.driveForward();
    // return new ParallelCommandGroup(
    //     swerveSubsystem.runExamplePath(),
    //     new RunCommand(() -> elevator.setHeightMeters(2), elevator));
    return getStupidFunniTestAuto();
  }

  public Command getStupidFunniTestAuto() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Pose2d startPose = new Pose2d(7.15, 6.5, new Rotation2d(Math.toRadians(180)));

    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometryAllianceAccounted(startPose)),
        new ParallelDeadlineGroup(
                swerveSubsystem.runPathManual("StartToJ"),
                elevator.setElevatorHeightCommand(
                    ReefHeight.L4.height + Elevator.kScoringOffsetHeight))
            .withTimeout(10),
        new WaitCommand(Seconds.of(0.5)),
        new ParallelDeadlineGroup(
                swerveSubsystem.runPathManual("JToIntake"),
                elevator.setElevatorHeightCommand(0.95) // Intaking height
                )
            .withTimeout(10),
        new WaitCommand(Seconds.of(0.5)),
        new ParallelDeadlineGroup(
                swerveSubsystem.runPathManual("IntakeToK"),
                elevator.setElevatorHeightCommand(
                    ReefHeight.L4.height + Elevator.kScoringOffsetHeight))
            .withTimeout(10),
        new WaitCommand(Seconds.of(0.5)),
        new ParallelDeadlineGroup(
                swerveSubsystem.runPathManual("KToIntake"),
                elevator.setElevatorHeightCommand(0.95) // Intaking height
                )
            .withTimeout(10),
        new WaitCommand(Seconds.of(0.5)),
        new ParallelDeadlineGroup(
                swerveSubsystem.runPathManual("IntakeToL"),
                elevator.setElevatorHeightCommand(
                    ReefHeight.L4.height + Elevator.kScoringOffsetHeight))
            .withTimeout(10));
  }
}
