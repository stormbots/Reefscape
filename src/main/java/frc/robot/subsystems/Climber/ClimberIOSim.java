// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0, 0),
          DCMotor.getNeoVortex(1));

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
}
