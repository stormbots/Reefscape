// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorPose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public final Elevator elevator = new Elevator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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

    driverController.a()
      .whileTrue(elevator.moveToPose(elevator.kStowed));
    SmartDashboard.putString("elevator/targetPose", "Stowed Up");
      

    driverController.b()
      .whileTrue(elevator.moveToPose(elevator.kFloorPickup));
      //.whileTrue(elevator.scoreAtPose(elevator.kFloorPickup));
      SmartDashboard.putString("elevator/targetPose", "Floor PickUp");

    driverController.x()
      .whileTrue(elevator.scoreAtPose(elevator.kStowedUp));
      SmartDashboard.putString("elevator/targetPose", "Stowed");

    driverController.y()
      .whileTrue(elevator.scoreAtPose(elevator.kL2));
      SmartDashboard.putString("elevator/targetPose", "L4");

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(exampleSubsystem);
    return new SequentialCommandGroup(
      //elevator.moveToAngle(180),
      elevator.moveToHeight(9).withTimeout(5),
      elevator.testMoveElevatorArmWithTrap(() -> 10)
      // elevator.moveToPose(elevator.kL2).withTimeout(5),
      // elevator.moveToHeight(20).withTimeout(5),
      // elevator.moveToAngle(90).withTimeout(5),
      // elevator.moveToAngle(-45).withTimeout(5),
      // elevator.moveToAngle(0).withTimeout(5),
      // elevator.scoreAtPose(elevator.kL2).withTimeout(5),
      // elevator.scoreAtPose(elevator.kStowed).withTimeout(5)

    );
  }
}
