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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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

    /** Track if we need to re-build the auto due to having ran it and done a field reset */
    private boolean requireRebuild=true;

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
        //New L4 autos
        autoChooser.addOption("L4 Basic Left Auto", this::leftL4CoralAuto);
        autoChooser.addOption("L4 Basic Right Auto", this::rightL4CoralAuto);
        autoChooser.addOption("L4 Basic Center Auto", this::centerL4CoralAuto);

        autoChooser.addOption("Multicoral Right Auto", this::rightMultiCoralAuto);
        autoChooser.addOption("Multicoral Left Auto", this::leftMultiCoralAuto);

        autoChooser.addOption("Drive Forward Score", this::driveForwardScore);
        autoChooser.addOption("Sliding Drive Foward Score", this::driverForwardLeftScore);
        
        autoChooser.addOption("v TEST AUTOS v",()->new InstantCommand());
        //test autos here
        autoChooser.addOption("testCenterAuto", this::basicCenterAutoTest);
        autoChooser.addOption("testLeftAuto", this::basicLeftAutoTest);
        //PUT UNTESTED AUTOS HERE; Drivers should not select these
        // autoChooser.addOption("1MeterNoTurn", ()->swerve.followPath("1Meter"));
        // autoChooser.addOption("1MeterTurn", ()->swerve.followPath("1MeterTurn"));
        // autoChooser.addOption("LongSpline", ()->swerve.followPath("LongSpline"));

    }

    /** Should be checked in Robot.java::autonomousInit
     *  This will return the selected auto, and *will* block the
     *  main thread if necessary for it to complete. However, this should 
     *  almost never happen, but is safely handled internally if it does
     */
    public Command getAutonomousCommand(){
        this.requireRebuild = true;
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
        var ready = selectedAutoFuture.isDone() && requireRebuild==false;
        var gyroReady = swerveSubsystem.isGyroConnected();
        SmartDashboard.putBoolean("AutoSelector/ready",ready);

        //If we haven't changed auto or alliance, nothing to do
        if(selectedAuto == autoChooser.getSelected() 
            && alliance == DriverStation.getAlliance().get()
            && requireRebuild==false
        ){
            return;
        }

        if( ! selectedAutoFuture.isDone() ){
          //Ideally we want to cancel the future, but it doesn't work properly
          //work around it by waiting until it's done first, then swap out the 
          //future
          return; 
        }

        //Save which one we're running now
        selectedAuto = autoChooser.getSelected();
        alliance = DriverStation.getAlliance().get();

        //No longer needs a rebuild
        requireRebuild=false;

        //Run the function that builds the desired auto in the background
        selectedAutoFuture = CompletableFuture.supplyAsync(selectedAuto);
    }

    ///////////////////////////////////////////////////
    /// Helpful snippets ////////////////////////////////
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

    public Command loadFromStation(){
      return Commands.sequence(
        new ParallelCommandGroup(
        swerveSubsystem.pathToCoralSource(),
        elevator.moveToPoseSafe(elevator.kStationPickup).until(()->elevator.isAtPosition(elevator.kStationPickup))
      ).until(scorer.isCoralInScorer).withTimeout(5),
      new ParallelCommandGroup(
        swerveSubsystem.stopCommand(),
        scorer.loadCoral(),
        elevator.moveToPoseSafe(elevator.kStationPickup)
      ).until(scorer.isCoralInScorer).withTimeout(5)
      );
    }

    public Command scoreAtL4(){
      return Commands.sequence(
        elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
        scorer.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90).until(elevator.isAtTargetAngle)
      )
      .alongWith(swerveSubsystem.stopCommand().withTimeout(4))
      ;
      //TODO deadline a stop command
    }
  
    ///////////////////////////////////////////////////
    /// PUT AUTOS HERE ////////////////////////////////
    ///////////////////////////////////////////////////

  public Command driveForwardScore(){
    return Commands.sequence(
      swerveSubsystem.driveCommandRobotRelative(()->-0.2,()-> 0.0, ()->0.0).withTimeout(2.25),
      scoreAtL4()
    );
  }

  public Command driverForwardLeftScore(){
    return Commands.sequence(
      swerveSubsystem.driveCommandRobotRelative(()->-0.2,()-> 0.0, ()->0.0).withTimeout(2.25),
      elevator.moveToPoseSafe(elevator.kL3).until(()->elevator.isAtPosition(elevator.kL3)),
      swerveSubsystem.driveCommandRobotRelative(()->-0.005,()->-0.05, ()->0.0).until(scorer.isBranchInRange),
      scoreAtL4()
    );
  }

  
  public Command centerL4CoralAuto(){
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // swerveSubsystem.resetOdometryAllianceManaged(new Pose2d(7.15, 4.18, new Rotation2d()));

    var path = "basicCenterAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    return Commands.sequence(
      swerveSubsystem.pidToCoralLeft().withTimeout(5),
      scoreAtL4()
    );
  }

  public Command leftL4CoralAuto(){
   var path = "basicLeftAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    return Commands.sequence(
      new ParallelCommandGroup(
      elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
      swerveSubsystem.pathToCoralLeft()),
      // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
      scoreAtL4()    
      );
  }

  public Command rightL4CoralAuto(){
    var path = "basicRightAuto";
    swerveSubsystem.setInitialPoseFromPath(path); //provide a sane default from pathplanner
    Timer.delay(5); //let vision set the precise location before building the path

    return Commands.sequence(
       // getUnfoldRobot().withTimeout(7),
      
        new ParallelCommandGroup(
        elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4)),
        swerveSubsystem.pathToCoralLeft()),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        scoreAtL4()    
        );
    
  }

  //done with pathfind+pid, might swap to actual pathplanner path if its too jank, works on both sides
  public Command leftMultiCoralAuto(){
    return Commands.sequence(
      leftL4CoralAuto(),
      loadFromStation(),
      new ParallelCommandGroup(
      swerveSubsystem.pathToCoralLeft(),
      elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4))
      ),
      //score coral
      swerveSubsystem.stopCommand().withTimeout(0.5),
      scoreAtL4(),
      loadFromStation(),
      new ParallelCommandGroup(
        swerveSubsystem.pathToCoralRight(),
        elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4))
      ),
      swerveSubsystem.stopCommand().withTimeout(0.5),
      scoreAtL4()
    );
  }
  //done with pathfind+pid, might swap to actual pathplanner path if its too jank, works both sides
  public Command rightMultiCoralAuto(){
    return Commands.sequence(
      rightL4CoralAuto(),

      loadFromStation(),
      new ParallelCommandGroup(
        swerveSubsystem.pathToCoralLeft(),
        elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4))
      ),
      //score coral
      scoreAtL4(),
      loadFromStation(),
      new ParallelCommandGroup(
        swerveSubsystem.pathToCoralRight(),
        elevator.moveToPoseSafe(elevator.kL4).until(()->elevator.isAtPosition(elevator.kL4))
      ),
      scoreAtL4()
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
      new WaitCommand(2),
      swerveSubsystem.pathToCoralSource(),
      new WaitCommand(2),
      swerveSubsystem.pathToCoralRight(),
      new WaitCommand(2),
      swerveSubsystem.pathToCoralSource(),
      new WaitCommand(2),
      swerveSubsystem.pathToCoralLeft()
    );

  }
  

}