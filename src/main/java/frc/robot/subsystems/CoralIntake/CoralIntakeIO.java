// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {}

  public default void updateInputs(CoralIntakeIOInputs inputs) {}
}
