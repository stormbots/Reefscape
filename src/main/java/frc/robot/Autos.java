// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabber;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Elevator.Elevator;

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
    Climber climber;
    CoralIntake intake;
    AlgaeGrabber algae;
    // Vision vision;

    public Autos(
        Swerve swerve,
        Elevator elevator,
        Climber climber,
        CoralIntake coralIntake,
        AlgaeGrabber algae
        // Vision vision
    ){
        this.swerveSubsystem = swerve;
        this.elevator = elevator;
        this.climber = climber;
        this.intake = coralIntake;
        // this.vision = vision;

        // Add options to our chooser; This could be done manually if we wanted
        SmartDashboard.putData("AutoSelector/chooser",autoChooser);
        autoChooser.setDefaultOption("Select Auto",()->new InstantCommand());
        //INSERT TESTED AUTOS HERE; Drivers wil use these.
        autoChooser.addOption("Basic Center Auto", this::basicCenterAuto);
        autoChooser.addOption("Basic Right Auto", this::basicRightAuto);
        autoChooser.addOption("Basic Left Auto", this::basicLeftAuto);

        autoChooser.addOption("v TEST AUTOS v",()->new InstantCommand());
        //PUT UNTESTED AUTOS HERE; Drivers should not select these
        autoChooser.addOption("1MeterNoTurn", ()->swerve.followPath("1Meter"));
        autoChooser.addOption("1MeterTurn", ()->swerve.followPath("1MeterTurn"));
        autoChooser.addOption("LongSpline", ()->swerve.followPath("LongSpline"));

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
          intake.setAngle(()-> 60.0).withTimeout(2),
          climber.setAngle(()->30.2)
        );
      }
    
  public Command basicCenterAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Blue){
        swerveSubsystem.resetOdometry(new Pose2d(10.0, 4.05, new Rotation2d(0.0)));
    }else{
      swerveSubsystem.resetOdometry(new Pose2d(7.6, 4.05, new Rotation2d(Degrees.of(180))));
    }
    return Commands.sequence(
        getUnfoldRobot(),
        swerveSubsystem.followPath("basicCenterAuto"),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL4).until(elevator.isAtTargetPosition),
        elevator.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }
  public Command basicLeftAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Blue){
        swerveSubsystem.resetOdometry(new Pose2d(10.0, 4.05, new Rotation2d(0.0)));
    }else{
      swerveSubsystem.resetOdometry(new Pose2d(7.6, 4.05, new Rotation2d(Degrees.of(180))));
    }
    return Commands.sequence(
        getUnfoldRobot(),
        swerveSubsystem.followPath("basicLeftAuto"),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL4).until(elevator.isAtTargetPosition),
        elevator.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }
  public Command basicRightAuto(){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Blue){
        swerveSubsystem.resetOdometry(new Pose2d(10.0, 4.05, new Rotation2d(0.0)));
    }else{
      swerveSubsystem.resetOdometry(new Pose2d(7.6, 4.05, new Rotation2d(Degrees.of(180))));
    }
    return Commands.sequence(
        getUnfoldRobot(),
        swerveSubsystem.followPath("basicRightAuto"),
        // elevator.scoreAtPoseSafe(elevator.kL4), //probably ok
        elevator.moveToPoseSafe(elevator.kL4).until(elevator.isAtTargetPosition),
        elevator.runCoralScorer(2500).withTimeout(1),
        elevator.moveToAngleTrap(()->90)
    );
  }
  


  

}