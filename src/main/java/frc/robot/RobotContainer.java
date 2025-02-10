// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeIOReal;
import frc.robot.subsystems.CoralIntake.CoralIntakeIOSim;
import frc.robot.subsystems.Elevator.ArmIOReal;
import frc.robot.subsystems.Elevator.ArmIOSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based i]
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private Swerve swerveSubsystem = new Swerve();
  private Elevator elevator;
  private CoralIntake coralIntake;
  
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  CommandXboxController driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.currentMode == Mode.real) {
      elevator = new Elevator("elevator", new ElevatorIOReal(), "coralArm", new ArmIOReal());
      coralIntake = new CoralIntake("coralIntake", new CoralIntakeIOReal());
    } else {
      elevator = new Elevator("elevator", new ElevatorIOSim(), "coralArm", new ArmIOSim());
      coralIntake = new CoralIntake("coralIntake", new CoralIntakeIOSim());
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

    swerveSubsystem.setDefaultCommand(
        swerveSubsystem.driveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    elevator.setDefaultCommand(elevator.setAngleRadiansCommand(Math.toRadians(90)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(exampleSubsystem);
    // return elevator.setAngleRadiansCommand(Math.toRadians(180));
    return getStupidFunniTestAuto();
    // return new RunCommand(()-> coralIntake.setPositionRadians(1.8), coralIntake);
  }

  public Command getStupidFunniTestAuto() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Pose2d startPose = new Pose2d(7.15, 6.5, new Rotation2d());

    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometryAllianceAccounted(startPose)),
        new ParallelDeadlineGroup(
                swerveSubsystem.runPathManual("StartToJ"),
                elevator.setElevatorHeightCommand(
                    ReefHeight.L4.height + Elevator.kScoringOffsetHeight))
            .withTimeout(10),
        new WaitCommand(Seconds.of(0.1)),
        intakeThenPlaceCommand("JToIntake", "IntakeToK"),
        intakeThenPlaceCommand("KToIntake", "IntakeToL"),
        intakeThenPlaceCommand("LToIntake", "IntakeToA"),
        intakeThenPlaceCommand("AToIntake", "IntakeToB")
      );
  }

  public Pose3d[] getFinalComponentPoses(){
    return new Pose3d[] {
      new Pose3d(0.3075,0,0.2525 + 0.05,new Rotation3d(0, -coralIntake.getPositionRadians(), 0)), // intake
      new Pose3d(0,-0.229,0.3805,new Rotation3d(Math.toRadians(0), 0, 0)), // climber,
      new Pose3d(-0.2535,0,0.7045,new Rotation3d(0, Math.toRadians(-90), 0)), //Algae Scorer
      new Pose3d(0,0.235,0.075+elevator.getHeight()/2,new Rotation3d(0, 0,0)), //Elevator first stage
      new Pose3d(-0.017, 0.15, 0.133+elevator.getHeight(), new Rotation3d(0, -elevator.getArmAngleRadians()+Math.toRadians(90), 0)), //Arm
      new Pose3d(-0.01+elevator.getArmCoordinates().getX(), 0.085, 0.57+elevator.getArmCoordinates().getY()-0.41, 
        new Rotation3d(0, -elevator.getArmAngleRadians()+Math.toRadians(90)-elevator.getCoralScorerAngleRadians(), 0)), //Coral Scorer
      new Pose3d(-0.018, 0.2, 0.1+elevator.getHeight(), new Rotation3d()) //Stage 2
    };
  }

  public Command intakeThenPlaceCommand(String intakePath, String scorePath){
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        swerveSubsystem.runPathManual(intakePath),
        new RunCommand(()->{elevator.setAngleRadians(Math.toRadians(-50)); elevator.setHeightMeters(0.7);}, elevator),
        coralIntake.setPositionRadiansCommand(-40)
      ).withTimeout(10),

        new WaitCommand(Seconds.of(0.1)),

      new ParallelDeadlineGroup(
        swerveSubsystem.runPathManual(scorePath),
        new RunCommand(()->{elevator.setAngleRadians(Math.toRadians(120)); elevator.setHeightMeters(Units.inchesToMeters(65));}, elevator),
        new WaitCommand(0.25).andThen(coralIntake.setPositionRadiansCommand(90))
      ).withTimeout(10),

      new WaitCommand(Seconds.of(0.1))
    );
  }
}
