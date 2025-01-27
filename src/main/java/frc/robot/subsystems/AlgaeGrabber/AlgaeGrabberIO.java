// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AlgaeGrabberIO {

  @AutoLog
  public static class AlgaeGrabberIOInputs {
    double armAngle = 0.0;
    double armVelocityRadPerSec = 0.0;
    double rollerVelocityRadPerSec = 0.0;
    double armAppliedVolts = 0.0;
    double rollerAppliedVolts = 0.0;
    double rollerCurrentDraw = 0.0;
    double armCurrentDraw = 0.0;
    boolean haveAlgae = false;
  }

  public default void updateInputs(AlgaeGrabberIOInputs inputs) {}

  public default void setVoltage(double armVolts, double rollerVolts) {}

  public default void setArmAngle(double angle) {}

  public default void setRollerVelocity(double velocity) {}
}
