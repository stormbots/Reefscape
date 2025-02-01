// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final ClimberVisualizer visualizer;
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;

    rateLimit.reset(io.getPosition());
    visualizer = new ClimberVisualizer("climber");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    visualizer.update(inputs.climberAbsoluteAngle);
  }

  public Command setAngle(double degrees) {
    return run(() -> io.setReference(degrees));
  }

  // public Command setBrakeMode(boolean brakeMode) {
  //   return run(() -> io.setBrakeMode(brakeMode));
  // }

  public Command prepareToClimb() {
    return run(() -> io.setReference(-60));
  }

  SlewRateLimiter rateLimit = new SlewRateLimiter(30);

  public Command climb() {
    return run(() -> io.setReference(43));
  }
}
