// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  SparkFlex climbMotor = new SparkFlex(30, MotorType.kBrushless);

  public ClimberIOReal() {}
}
