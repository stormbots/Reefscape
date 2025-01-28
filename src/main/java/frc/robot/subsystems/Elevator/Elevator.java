// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final String name;
  private final ElevatorIO io;

  private final ElevatorVisualizer bruhVisualizer;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  /** Creates a new Ele`vator. */
  public Elevator(String name, ElevatorIO io) {
    this.name = name;
    this.io = io;

    bruhVisualizer = new ElevatorVisualizer("bruh");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    bruhVisualizer.update(inputs.heightMeters);
  }

  public Command runElevator(double volts) {
    return startEnd(() -> io.setVoltage(volts), () -> io.stop());
  }
}
