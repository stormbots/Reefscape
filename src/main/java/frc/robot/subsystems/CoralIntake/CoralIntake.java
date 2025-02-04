// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  private final String name;
  private final CoralIntakeIO io;

  private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

  /** Creates a new CoralIntake. */
  public CoralIntake(String name, CoralIntakeIO io) {
    this.name = name;
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void setPositionRadians(double radians) {
    PIDController pid = new PIDController(5, 0, 0);
    double error = radians - inputs.relativePositionRads;
    Logger.recordOutput("coralIntake/errorDeg", Math.toDegrees(error));

    io.setVoltage(-pid.calculate(error));
    Logger.recordOutput("coralIntake/pidOutput", pid.calculate(error));
  }

  public void setVoltage(double volts){
    io.setVoltage(volts);
  }

  public double getPositionRadians(){
    return inputs.relativePositionRads;
  }

  public Command setPositionRadiansCommand(double radians){
    return new RunCommand(()->setPositionRadians(radians), this);
  }
}
