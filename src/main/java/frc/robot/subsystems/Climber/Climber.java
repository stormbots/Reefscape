// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final ClimberVisualizer visualizer;
  SlewRateLimiter rateLimit = new SlewRateLimiter(15);
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;

    rateLimit.reset(getPosition());
    visualizer = new ClimberVisualizer("climber");

    setDefaultCommand(holdPosition(this::getPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    visualizer.update(inputs.climberAbsoluteAngle);
  }

  public Command holdPosition(DoubleSupplier degrees) {
    return startRun(
      () -> io.setReference(degrees.getAsDouble()),
      ()-> {}
    );
  }


  public Command setAngle(double degrees) {
    return startRun(
      ()-> rateLimit.reset(getPosition()),
      () -> io.setReference(rateLimit.calculate(degrees))
      );
    // return run(() -> io.setReference(degrees));

  }

  public Command setBrakeMode() {
    return run(() -> io.setBrakeMode());
  }

  public Command prepareToClimb() {
    return setAngle(-60);
  }

  

  public Command climb() {
    return setAngle(60);
  }

  public double getPosition(){
    var angle = io.getPosition();
    if(angle>180) angle = angle-360;
    return angle;
  }
}
