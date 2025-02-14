// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Climber.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorPose;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based i]
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public final Climber climber = new Climber();
  public final CoralIntake intake = new CoralIntake();

  // private final Tabi tabi = new Tabi();

  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();
  public final Elevator elevator = new Elevator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // private final CommandXboxController operatorController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public AHRS navxGyro = new AHRS(NavXComType.kMXP_SPI);

  Swerve swerveSubsystem = new Swerve();
  private final Vision Vision = new Vision(swerveSubsystem, navxGyro);

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
    // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

    // Expected algae control stuff
    // driverController.x().whileTrue(algaeGrabber.intakeAlgaeFromFloor());
    // driverController.y().whileTrue(algaeGrabber.scoreProcessor());
    // driverController.a().whileTrue(algaeGrabber.prepareToShoot());
    // driverController.b().whileTrue(algaeGrabber.scoreInNetEzMode());

    driverController.a()
      .whileTrue(elevator.scoreAtPose(elevator.kL1));
      SmartDashboard.putString("elevator/targetPose", "L1");
      

    driverController.b()
      .whileTrue(elevator.scoreAtPose(elevator.kL2));
      SmartDashboard.putString("elevator/targetPose", "L2");

    driverController.x()
      .whileTrue(elevator.scoreAtPose(elevator.new ElevatorPose(20, -40, 0)));
      SmartDashboard.putString("elevator/targetPose", "L3");

    driverController.y()
      .whileTrue(elevator.scoreAtPose(elevator.kL4));
      SmartDashboard.putString("elevator/targetPose", "L4");

    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(()->-driverController.getLeftY()/4.0, ()->-driverController.getLeftX()/4.0, ()->-driverController.getRightX()/4.0)
    );

   /*  driverController.b().whileTrue(
    swerveSubsystem.pathToPose(new Pose2d())
    );*/
    driverController.start().onTrue(swerveSubsystem.resetGyro());

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

    return new SequentialCommandGroup(
      new WaitCommand(5),
      climber.setAngle(()->0)
    );
  }

  public Pose3d[] getFinalComponentPoses(){
    //Original, use once methods get implemented
    // return new Pose3d[] {
    //   new Pose3d(0.3075,0,0.2525 + 0.05,new Rotation3d(0, -coralIntake.getPositionRadians(), 0)), // intake
    //   new Pose3d(0,-0.229,0.3805,new Rotation3d(Math.toRadians(0), 0, 0)), // climber,
    //   new Pose3d(-0.2535,0,0.7045,new Rotation3d(0, Math.toRadians(-90), 0)), //Algae Scorer
    //   new Pose3d(0,0.235,0.075+elevator.getHeight()/2,new Rotation3d(0, 0,0)), //Elevator first stage
    //   new Pose3d(-0.017, 0.15, 0.133+elevator.getHeight(), new Rotation3d(0, -elevator.getArmAngleRadians()+Math.toRadians(90), 0)), //Arm
    //   new Pose3d(-0.01+elevator.getArmCoordinates().getX(), 0.085, 0.57+elevator.getArmCoordinates().getY()-0.41, 
    //     new Rotation3d(0, -elevator.getArmAngleRadians()+Math.toRadians(90)-elevator.getCoralScorerAngleRadians(), 0)), //Coral Scorer
    //   new Pose3d(-0.018, 0.2, 0.1+elevator.getHeight(), new Rotation3d()) //Stage 2
    // };

    //defaulted w/o subsytems
    return new Pose3d[] {
      new Pose3d(0.3075,0,0.2525 + 0.05,new Rotation3d(0, -intake.getAngle().in(Units.Radians), 0)), // intake
      new Pose3d(0,-0.229,0.3805,new Rotation3d(-Math.toRadians(climber.getAngle().in(Radians)+90), 0, 0)), // climber,
      new Pose3d(-0.2535,0,0.7045,new Rotation3d(0, Math.toRadians(algaeGrabber.getAngle()), 0)), //Algae Scorer
      new Pose3d(0,0.235,0.075+elevator.getHeight().in(Units.Meters)/2,new Rotation3d(0, 0,0)), //Elevator first stage
      new Pose3d(-0.017, 0.15, 0.133+elevator.getHeight().in(Units.Meters), new Rotation3d(0, -elevator.getArmAngle().in(Radians)+Math.toRadians(90), 0)), //Arm
      new Pose3d(-0.01, 0.085, 0.57+0.41-0.41, 
        new Rotation3d(0, 0, 0)), //Coral Scorer
        new Pose3d(-0.018, 0.2, 0.1+elevator.getHeight().in(Units.Meters), new Rotation3d()) //Stage 2
    };
  }
}
