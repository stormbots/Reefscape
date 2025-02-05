// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO{
    //All these values are completely wrong...
    private double appliedVoltage = 0.0;

    private final DCMotor model = DCMotor.getNeoVortex(1);
    private static final double gearing = 67.0;

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        model, 
        gearing, 
        0.1, 
        0.4, 
        Math.toRadians(-90), 
        Math.toRadians(180), 
        true, 
        90,
        new double[]{0,0}
    );

    public ArmIOSim(){}

    public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(0.02);

    inputs.appliedVoltage = appliedVoltage;
    inputs.armRelativePositionRads = sim.getAngleRads();
    inputs.armAbsolutePositionRads = sim.getAngleRads();
    inputs.coralScorerPositionRads = -(sim.getAngleRads()-Math.toRadians(90))/2.0;
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
