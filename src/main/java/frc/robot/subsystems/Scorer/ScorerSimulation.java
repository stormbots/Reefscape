// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scorer;


import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class ScorerSimulation {

  SparkFlex scoringMotor;

  SparkFlexSim simScoringMotor;
  
  public ScorerSimulation(
    SparkFlex scoringMotor
  ){
    this.scoringMotor = scoringMotor;
    simScoringMotor = new SparkFlexSim(this.scoringMotor, DCMotor.getNeoVortex(1));
  }
  
  FlywheelSim simScorer = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), 0.00002016125, 1
    ),
    DCMotor.getNeoVortex(1)
  );

  public void update() {
    // TODO Auto-generated method stub
    var vbus = 12;
    var dt = 0.02;

    simScorer.setInputVoltage(simScoringMotor.getAppliedOutput()*vbus);
    simScorer.update(dt);
    simScoringMotor.iterate(simScorer.getAngularVelocityRPM(), vbus, dt);
  }

}
