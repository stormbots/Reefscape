// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

/** Add your docs here. */
public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    double armAbsoluteAngle = 0.0;
    double armRelativeAngle = 0.0;
    double rollerVelocity = 0.0;
    double rollerVoltage = 0.0;
    double rollerCurrentDraw = 0.0;
    double armVoltage = 0.0;
    double armCurrentDraw = 0.0;
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public default void setArmAngle(double angle) {}

  public default void setRollerVelocity(double velocity) {}

  public default double getArmAngle() { return 0;}

  public default double getRollerVelocity() { return 0;}

  public default void setRelativeEncoderPosition() {}

  public default void configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {}

  public default void configureAsync(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {}
}
