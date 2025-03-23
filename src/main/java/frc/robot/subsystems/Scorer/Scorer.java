// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Scorer;


import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeter;

import java.util.Optional;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stormbots.LaserCanWrapper;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;


public class Scorer extends SubsystemBase {
  Optional<ScorerSimulation> sim = Optional.empty();
  
  SparkFlex motor = new SparkFlex(12, MotorType.kBrushless);

  private final Distance kNoCoralDistance = Millimeter.of(50);
  LaserCanWrapper laserCan = new LaserCanWrapper(22)
    .configureShortRange()
    .setThreshhold(kNoCoralDistance)
    ;
  LaserCanWrapper branchDetector = new LaserCanWrapper(21)
  .configureShortRange()
  .setThreshhold(Inches.of(12.0));

  public Scorer() {
    if(Robot.isSimulation()) sim = Optional.of(new ScorerSimulation(motor));

    //Set up motor configs
    motor.configure(ScorerMotorConfigs.getScorerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(0.1); //Wait until motors are happy and providing us correct data

    setDefaultCommand(realignCoral());
  }

  public void periodic(){
    // SmartDashboard.putNumber("scorer/rpm", motor.getEncoder().getVelocity());
    SmartDashboard.putBoolean("ReefinRange", isBranchInRange.getAsBoolean());
  }

  public Trigger isCoralInScorer = laserCan.isBreakBeamTripped;
  public Trigger isBranchInRange = branchDetector.isBreakBeamTripped.debounce(0.03);

  //TODO: Test
  public Trigger isAlgaeInScorer = new Trigger(()->
    branchDetector.getDistanceOptional().orElse(Inches.of(99)).in(Inch) <= 2.0
  );

  

  private void setScorerSpeed(double speed) {
    motor
        .getClosedLoopController()
        .setReference(speed, ControlType.kVelocity);
    // , ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
    // coralOutMotor.setVoltage(SmartDashboard.getNumber("elevator/scorerSpeed", 0.000000000001)); //Temporary, just to make coral pickup work..., bruh
    // setpoint.speed = speed;
  }

  public Command runCoralScorer(double speed){
    return run(()->setScorerSpeed(speed));
    // return run(()->coralOutMotor.setVoltage(6))
  }


  public Command realignCoral(){
    return Commands.sequence(
      run(()->setScorerSpeed(-800)).onlyWhile(isCoralInScorer),
      new InstantCommand(()->motor.getEncoder().setPosition(0)),
      run(()->motor.getClosedLoopController().setReference(5, ControlType.kPosition, ClosedLoopSlot.kSlot1))
    );
  }

  public void stop(){
    motor.set(0);
  }

  public Command defaultCommand(){
    return new ConditionalCommand(
      holdAlgae(), 
      conditionalRealignCoral(), 
      isAlgaeInScorer);
  }

  private Command conditionalRealignCoral(){
    return new ConditionalCommand(
      realignCoral(), 
      run(()->motor.stopMotor()), 
      isCoralInScorer);
  }

  ////////////////////////////////////////////////////////////////////////
  /// Make some cleaner functions so autos and buttons are easier and more clear. 
  /// ////////////////////////////////////////////////////////////////////
  public Command scoreCoral(){
    return new SequentialCommandGroup(
      //Always run the intake for long enough to do it
      runCoralScorer(2500).withTimeout(0.5)
      );
  }

  public Command scoreCoralL1(){
    return new SequentialCommandGroup(
    runCoralScorer(1250).withTimeout(0.5)
    );
  }


  public Command holdAlgae(){
    return new SequentialCommandGroup(
      //Always run the intake for long enough to do it
      runCoralScorer(-2500) //TODO fix me
      );
  }

  public Command dropAlgae(){
    return new SequentialCommandGroup(
      runCoralScorer(500)
    );
   
  }

  public Command loadCoral(){
    return new SequentialCommandGroup(
      //TODO: Put the right things here. 
      runCoralScorer(2500).until(isCoralInScorer),
      realignCoral()
    ).finallyDo(this::stop);
  }

  @Override
  public void simulationPeriodic(){
    if(sim.isEmpty())return;
    sim.get().update();
  }

}

