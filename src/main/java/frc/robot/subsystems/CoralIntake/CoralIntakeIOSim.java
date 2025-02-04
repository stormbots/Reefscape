// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class CoralIntakeIOSim implements CoralIntakeIO{

    private double appliedVoltage = 0.0;

    private final DCMotor model = DCMotor.getNeoVortex(1);
    private static final double gearing = 67.0;

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        model, 
        gearing, 
        0.6, 
        0.37, 
        Math.toRadians(-50), 
        Math.toRadians(90), 
        true, 
        Math.toRadians(90),
        new double[]{0,0}
    );

    public CoralIntakeIOSim(){}

    @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(0.02);

    inputs.appliedVoltage = appliedVoltage;
    inputs.relativePositionRads = sim.getAngleRads();
    inputs.absolutePositionRads = sim.getAngleRads();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }    
}
