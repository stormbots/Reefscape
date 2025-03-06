// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Scorer;


import static edu.wpi.first.units.Units.Millimeter;

import java.util.Optional;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stormbots.LaserCanWrapper;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;


public class Scorer extends SubsystemBase {
  public boolean isHomed = false;
  public int ROLLERSTALLCURRENT;

  Optional<ScorerSimulation> sim = Optional.empty();
  
  SparkFlex coralOutMotor = new SparkFlex(12, MotorType.kBrushless);

  private final Distance kNoCoralDistance = Millimeter.of(50);
  LaserCanWrapper laserCan = new LaserCanWrapper(22)
    .configureShortRange()
    .setThreshhold(kNoCoralDistance)
    ;

  public Scorer() {
    if(Robot.isSimulation()) sim = Optional.of(new ScorerSimulation(coralOutMotor));

    //Set up motor configs
    coralOutMotor.configure(ScorerMotorConfigs.getScorerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(0.1); //Wait until motors are happy and providing us correct data


    // setDefaultCommand(holdPosition());
    setDefaultCommand(run(()->{
      coralOutMotor.stopMotor();
    }));

  }

  public void periodic(){
    SmartDashboard.putNumber("scorer/rpm", coralOutMotor.getEncoder().getVelocity());
  }

  public Trigger isCoralInScorer = laserCan.isBreakBeamTripped;

  public Trigger isCoralScorerStalled = new Trigger( () -> {
    return coralOutMotor.getOutputCurrent() > ROLLERSTALLCURRENT;
  }).debounce(0.1);


  SlewRateLimiter slewRateAngle = new SlewRateLimiter(60);
  

  private void setScorerSpeed(double speed) {
    coralOutMotor
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

  public Command pidScorerBack(){
    return run(()->{})//coralOutMotor.getEncoder().setPosition(0))
    .andThen(run(()->coralOutMotor.getClosedLoopController().setReference(-2, ControlType.kPosition, ClosedLoopSlot.kSlot1)));
  }

  public Command realignCoralScorer(){
    return Commands.sequence(
      run(()->setScorerSpeed(-1200)).onlyWhile(isCoralInScorer),
      new InstantCommand(()->coralOutMotor.getEncoder().setPosition(0)),
      run(()->coralOutMotor.getClosedLoopController().setReference(2, ControlType.kPosition, ClosedLoopSlot.kSlot1))
    );
  }

  @Override
  public void simulationPeriodic(){
    if(sim.isEmpty())return;
    sim.get().update();
  }

}

