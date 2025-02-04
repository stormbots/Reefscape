// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
  private DCMotorSim sim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0, 0),
          DCMotor.getNeoVortex(1));
  
  private SparkFlex sparkFlex = new SparkFlex(6, MotorType.kBrushless);
  private DCMotor dcmotor = DCMotor.getNeoVortex(1);
  private SparkFlexSim sparkSim = new SparkFlexSim(sparkFlex, dcmotor);
        
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(.002);

    inputs.climberRelativeAngle = Units.rotationsToDegrees(sim.getAngularPositionRotations());
    inputs.climberVoltage = sim.getInputVoltage();
    inputs.climberCurrentDraw = sim.getCurrentDrawAmps();
  }

  @Override
  public void setReference(double angle) {
    sim.setAngle(Units.degreesToRadians(angle));
  }

  @Override
  public void setIdleMode(IdleMode idleMode) {
    configureAsync(new SparkFlexConfig().idleMode(idleMode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  @Override
  public void configureAsync(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    motor.configureAsync(config, resetMode, persistMode);
  }

  @Override
  public void configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    motor.configure(config, resetMode, persistMode);
  }
}
