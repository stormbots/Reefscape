// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armRelativePositionRads = 0.0;
    public double armAbsolutePositionRads = 0.0;
    public double coralScorerPositionRads = 0.0; //Relative to arm
    public double appliedVoltage = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double volts){}

  public default void stop(){}
}
