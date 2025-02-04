// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
  private final DCMotor gearbox;
  private final SparkFlex flex;
  private final SparkFlexSim sim;
  private final SingleJointedArmSim climberSim;

  // private double appliedVolts = 0.0;
  public ClimberIOSim() {
    gearbox = DCMotor.getNeoVortex(1);
    flex = new SparkFlex(9, MotorType.kBrushless);
    sim = new SparkFlexSim(flex, gearbox);
    climberSim = new SingleJointedArmSim(gearbox, 360.0, 0.005, 0.5, 0.0, 2*(Math.PI), true, 0.0);
  
    
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.setInput(sim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    climberSim.update(0.02);
    sim.iterate(Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));
    
  }

  @Override
  public void setReference(double angle) {
    
  }

  // @Override
  // public double getPosition() {
    
  // }
}
