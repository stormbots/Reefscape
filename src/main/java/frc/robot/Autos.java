// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabber;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Scorer.Scorer;

/** Add your docs here. */
public class Autos {
    /** The future value we care about */
    private CompletableFuture<Command> selectedAutoFuture = CompletableFuture.supplyAsync(()->new InstantCommand());

    /** Alliance color; Must be properly set before we can build autos, so we do that */
    private Alliance alliance = DriverStation.Alliance.Red;

    SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

    /** The value last selected on the chooser.  */
    private Supplier<Command> selectedAuto = ()->new InstantCommand();

    //Pass in subsystems so we can access them easily
    Swerve swerveSubsystem;
    Elevator elevator;
    Scorer scorer;
    Climber climber;
    AlgaeGrabber algae;
    // Vision vision;

    public Autos(
        Swerve swerve,
        Elevator elevator,
        Scorer scorer,
        Climber climber,
        AlgaeGrabber algae
        // Vision vision
    ){
        this.swerveSubsystem = swerve;
        this.elevator = elevator;
        this.scorer = scorer;
        this.climber = climber;
        // this.vision = vision;

        // Add options to our chooser; This could be done manually if we wanted
        SmartDashboard.putData("AutoSelector/chooser",autoChooser);
        autoChooser.setDefaultOption("Select Auto",()->new InstantCommand());
        //INSERT TESTED AUTOS HERE; Drivers wil use these.
        // autoChooser.addOption("drive first left auto", this::driveFirstLeftAuto);
        // autoChooser.addOption("drive first right auto", this::driveFirstRightAuto);

        autoChooser.addOption("L4 Basic Center Auto", this::basicCenterAutoL4);
        autoChooser.addOption("Algae Basic Center Auto", this::basicCenterAutoAlgaeGrab);
        autoChooser.addOption("Far Algae Basic Center Auto", this::basicCenterAutoFarAlgaeGrab);
        // autoChooser.addOption("Less Basic Center Auto", this::lessBasicCenterAuto);
        // autoChooser.addOption("Basic Right Auto", this::basicRightAuto);
        // autoChooser.addOption("Less Basic Right Auto", this::lessBasicRightAuto);
        autoChooser.addOption("Algae Basic Right Auto", this::basicRightAutoAlgaeGrab);
        autoChooser.addOption("Far Algae Basic Right Auto", this::basicRightAutoFarAlgaeGrab);
        // autoChooser.addOption("Basic Left Auto", this::basicLeftAuto);
        // autoChooser.addOption("Less Basic Left Auto", this::lessBasicLeftAuto);
        autoChooser.addOption("Algae Basic Left Auto", this::basicLeftAutoAlgaeGrab);
        autoChooser.addOption("Far Algae Basic Left Auto", this::basicLeftAutoFarAlgaeGrab);
        autoChooser.addOption("Drive forward for 5 seconds :'(", this::driveForwardAuto);

        autoChooser.addOption("v TEST AUTOS v",()->new InstantCommand());
        //test autos here
        autoChooser.addOption("testCenterAuto", this::basicCenterAutoTest);
        autoChooser.addOption("testLeftAuto", this::basicLeftAutoTest);
        //PUT UNTESTED AUTOS HERE; Drivers should not select these
        // autoChooser.addOption("1MeterNoTurn", ()->swerve.followPath("1Meter"));
        // autoChooser.addOption("1MeterTurn", ()->swerve.followPath("1MeterTurn"));
        // autoChooser.addOption("LongSpline", ()->swerve.followPath("LongSpline"));

        autoChooser.addOption("v EXAMPLES v",()->new InstantCommand());
        autoChooser.addOption("2024 Example Amp", this::ampTwoNote);
        autoChooser.addOption("2024 Example Source", this::sourceTwoNote);
    }

    /** Should be checked in Robot.java::autonomousInit
     *  This will return the selected auto, and *will* block the
     *  main thread if necessary for it to complete. However, this should 
     *  almost never happen, but is safely handled internally if it does
     */
    public Command getAutonomousCommand(){
      //TODO: mark command as "ran" and rebuild in periodic
        try{
             return selectedAutoFuture.get();
        }
        catch(Exception e){
            //If the auto cannot build, then we get an error and print it.
            System.err.println("Failed to build auto command ");
            System.err.println(e);
        }
        return new InstantCommand();
    }

    /**
     * The poll operation that watches for updates that affect autos.
     * This should be called in robot.java::DisabledPeriodic. 
     */
    void periodic(){
        //make sure we have our team color: It's necessary to guarantee this before building autos
        if(DriverStation.getAlliance().isEmpty()) return; 

        //Make sure either a default exists or a option was selected
        if(autoChooser.getSelected() == null) return;

        //Just show the status of our current build on the dashboard
        SmartDashboard.putBoolean("AutoSelector/ready",selectedAutoFuture.isDone());

        //If we haven't changed auto or alliance, nothing to do
        if(selectedAuto == autoChooser.getSelected() 
            && alliance == DriverStation.getAlliance().get()
        ){
            return;
        }

        //Save which one we're running now
        selectedAuto = autoChooser.getSelected();
        alliance = DriverStation.getAlliance().get();

        //cancel current computation and start the new one.
        selectedAutoFuture.cancel(true);

        //Run the function that builds the desired auto in the background
        selectedAutoFuture = CompletableFuture.supplyAsync(selectedAuto);
    }

    // Example autos that have *super* long build times thanks to timer.delay 
    private Command ampTwoNote(){
        System.out.println("starting auto build");
        Timer.delay(3);
        System.out.println("built amp auto");
        return new InstantCommand();
    }
    private Command sourceTwoNote(){
        System.out.println("starting auto build");
        Timer.delay(1.5);
        System.out.println("built source auto");
        return new InstantCommand();
    }

    ///////////////////////////////////////////////////
    /// PUT AUTOS HERE ////////////////////////////////
    ///////////////////////////////////////////////////
    /// 
    public Command getUnfoldRobot() {
        return Commands.sequence(
          elevator.moveToPoseUnchecked(elevator.kStowedUp).until(elevator.isAtTargetPosition),
          new ParallelCommandGroup(
            climber.setAngle(()->30.2)
          )
        );
      }
    
  public Command basicCenterAutoL4(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));

    var path = "basicCenterAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    return Commands.sequence(
      //new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath(path),
      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
      new WaitCommand(0.5),
      scorer.runCoralScorer(2500).withTimeout(1),
      elevator.moveToAngleTrap(()->90)
    ).andThen(
      //grab algae and shoot, not yet possible need elev and alg changes
    );
  }

  public Command basicLeftAutoL4(){
    var path = "basicLeftAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    return Commands.sequence(
        getUnfoldRobot().withTimeout(7),
        swerveSubsystem.followPath("basicLeftAuto"),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
        elevator.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90),
        new WaitCommand(3)
    ).andThen(
      //moving to coral source then scoring again command sequence
        coralSourceScoreLeft()
    );
  }

  public Command coralSourceScoreLeft(){
    return Commands.sequence(
      swerveSubsystem.pathToCoralSource(),//.withTimeout(3),
      swerveSubsystem.pidToCoralSource().withTimeout(1),
     // elevator.moveToPoseSafe(elevator.kStationPickup).until(()->elevator.isAtPosition(elevator.kStationPickup)),
      //elevator coral scorer intake command
      swerveSubsystem.pathToCoralLeft(),//.withTimeout(3),
      swerveSubsystem.pidToCoralLeft().withTimeout(1)
     // elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
     // elevator.runCoralScorer(2500).withTimeout(1),
     // elevator.moveToAngleTrap(()->90)
    );
  }

  public Command basicCenterAutoTest(){
    
    var path = "basicCenterAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    //sequence that only moves, no elevator or climber stuff
    return Commands.sequence(
      swerveSubsystem.followPath(path)
    );
  }
  public Command basicLeftAutoTest(){

    var path = "basicLeftAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    //sequence that only moves, no elevator or climber stuff
    return Commands.sequence(
      swerveSubsystem.followPath(path),
      swerveSubsystem.pidToCoralLeft().withTimeout(1)
    ).andThen(
      coralSourceScoreLeft()
    );
  }

  public Command basicCenterAutoAlgaeGrab(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
    return Commands.sequence(
      //new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath("basicCenterAuto"),
      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          elevator.moveToPoseSafe(elevator.kL2Algae).until(()->elevator.isAtPosition(elevator.kL2Algae)),
          new WaitCommand(1),
          elevator.moveToAngleTrap(()->90)
        ),
        scorer.runCoralScorer(-2500)
      )
    );
  }

  public Command basicCenterAutoFarAlgaeGrab(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
    return Commands.sequence(
      //new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath("basicCenterAuto"),
      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          elevator.moveToPoseSafe(elevator.kL2AlgaeFar).until(()->elevator.isAtPosition(elevator.kL2AlgaeFar)),
          new WaitCommand(1),
          elevator.moveToAngleTrap(()->90)
        ),
        scorer.runCoralScorer(-2500)
      )
    );
  }

  // public Command lessBasicCenterAuto(){
  //   // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  //   // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
  //   return Commands.sequence(
  //     new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()))),
  //     getUnfoldRobot().withTimeout(7),
  //     swerveSubsystem.followPath("basicCenterAuto"),
  //     swerveSubsystem.pathToCoralRight().withTimeout(3),
  //     // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
  //     elevator.moveToPoseSafe(elevator.kL2).until(()->elevator.isAtPosition(elevator.kL2)),
  //     new WaitCommand(0.5),
  //     scorer.runCoralScorer(2500).withTimeout(1),
  //     elevator.moveToAngleTrap(()->90)
  //   );
  // }

  public Command basicLeftAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return Commands.sequence(
    //new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 6.15, new Rotation2d()))),
        getUnfoldRobot().withTimeout(7),
        swerveSubsystem.followPath("basicLeftAuto"),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL3).until(()->elevator.isAtPosition(elevator.kL3)),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }

  public Command lessBasicLeftAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return Commands.sequence(
    //new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 6.15, new Rotation2d()))),
        getUnfoldRobot().withTimeout(7),
        swerveSubsystem.followPath("basicLeftAuto"),
        swerveSubsystem.pidToCoralRight().withTimeout(3),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL2).until(()->elevator.isAtPosition(elevator.kL2)),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }

  public Command driveFirstLeftAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return Commands.sequence(
    new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 6.15, new Rotation2d()))),
        swerveSubsystem.followPath("basicLeftAuto"),
        getUnfoldRobot().withTimeout(7),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL3).until(()->elevator.isAtPosition(elevator.kL3)),
        new WaitCommand(0.5),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }

  public Command basicLeftAutoAlgaeGrab(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
    return Commands.sequence(
      new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 6.15, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath("basicLeftAuto"),
            // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          elevator.moveToPoseSafe(elevator.kL2Algae).until(()->elevator.isAtPosition(elevator.kL2Algae)),
          new WaitCommand(1),
          elevator.moveToAngleTrap(()->90)
        ),
        scorer.runCoralScorer(-2500)
      )
    );
  }

  public Command basicLeftAutoFarAlgaeGrab(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
    return Commands.sequence(
      new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 6.15, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath("basicLeftAuto"),
      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          elevator.moveToPoseSafe(elevator.kL2AlgaeFar).until(()->elevator.isAtPosition(elevator.kL2AlgaeFar)),
          new WaitCommand(1),
          elevator.moveToAngleTrap(()->90)
        ),
        scorer.runCoralScorer(-2500)
      )
    );
  }

  public Command basicRightAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return Commands.sequence(
        new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 1.88, new Rotation2d()))),
        getUnfoldRobot(),
        swerveSubsystem.followPath("basicRightAuto"),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL3).until(()->elevator.isAtPosition(elevator.kL3)),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }

  public Command lessBasicRightAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return Commands.sequence(
        new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 1.88, new Rotation2d()))),
        getUnfoldRobot(),
        swerveSubsystem.followPath("basicRightAuto"),
        swerveSubsystem.pidToCoralLeft().withTimeout(3),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL2).until(()->elevator.isAtPosition(elevator.kL2)),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }

  public Command driveFirstRightAuto(){
    return Commands.sequence(
        new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 1.88, new Rotation2d()))),
        swerveSubsystem.followPath("basicRightAuto"),
        getUnfoldRobot(),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL3).until(()->elevator.isAtPosition(elevator.kL3)),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }

  public Command basicRightAutoAlgaeGrab(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
    return Commands.sequence(
      new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 1.88, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath("basicRightAuto"),
            // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          elevator.moveToPoseSafe(elevator.kL2Algae).until(()->elevator.isAtPosition(elevator.kL2Algae)),
          new WaitCommand(1),
          elevator.moveToAngleTrap(()->90)
        ),
        scorer.runCoralScorer(-2500)
      )
    );
  }

  public Command basicRightAutoFarAlgaeGrab(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));
    return Commands.sequence(
      new InstantCommand(()->swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 1.88, new Rotation2d()))),
      getUnfoldRobot().withTimeout(7),
      swerveSubsystem.followPath("basicRightAuto"),
      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          elevator.moveToPoseSafe(elevator.kL2AlgaeFar).until(()->elevator.isAtPosition(elevator.kL2AlgaeFar)),
          new WaitCommand(1),
          elevator.moveToAngleTrap(()->90)
        ),
        scorer.runCoralScorer(-2500)
      )
    );
  }
  
  public Command driveForwardAuto(){
    return swerveSubsystem.driveCommandRobotRelative(()->-0.5, ()->0.0, ()->0.0).withTimeout(2);
  }


  

}