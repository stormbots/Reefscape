// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private double appliedVoltage = 0.0;

  private static final DCMotor modelMotor = DCMotor.getNeoVortex(2);
  private static final double reduction = 3.0; // Fix please
  private static final double drumRadius = Units.inchesToMeters(0.912);

  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            modelMotor,
            reduction,
            3.11845,
            drumRadius,
            Units.inchesToMeters(10),
            Units.inchesToMeters(100),
            true,
            Units.inchesToMeters(10),
            new double[] {0.0, 0.0});
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(0.02);

    inputs.appliedVoltage = appliedVoltage;
    inputs.heightMeters = sim.getPositionMeters();
    inputs.velocityMPS = sim.getVelocityMetersPerSecond();
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
