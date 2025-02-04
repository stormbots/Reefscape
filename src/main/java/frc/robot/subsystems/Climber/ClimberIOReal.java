// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  SparkFlex climbMotor = new SparkFlex(9, MotorType.kBrushless);
  // 0 deg, 50.711
  // 88 deg, 153.705
  // lower softlimit 300 deg (abs) around inside bot
  // upper softlimit/stow 43 (abs) deg
  // motor invert true, absolute encoder fine

  // TODO: Fix wrapping issues for softlimits

  public ClimberIOReal() {
    var config = new SparkFlexConfig();
    // config.encoder.positionConversionFactor((360 / (153.705 - 50.711 / 88.0)));
    config.encoder.positionConversionFactor(1);
    config.absoluteEncoder.positionConversionFactor(360);

    config.inverted(true);
    config
        .closedLoop
        .outputRange(-0.5, 0.5)
        .p(1 / 30.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        // .positionWrappingInputRange(0, 360);
        .positionWrappingInputRange(-180, 180);
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(5).voltageCompensation(12.0);
    config.absoluteEncoder.inverted(false);
    config
        .softLimit
        .forwardSoftLimit(43)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(-60)
        .reverseSoftLimitEnabled(false);

    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    climbMotor.getEncoder().setPosition(climbMotor.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberAbsoluteAngle = climbMotor.getAbsoluteEncoder().getPosition();
    inputs.climberRelativeAngle = climbMotor.getEncoder().getPosition();
    inputs.climberVoltage = climbMotor.getAppliedOutput() * climbMotor.getBusVoltage();
    inputs.climberCurrentDraw = climbMotor.getOutputCurrent();
  }

  // @Override
  // public void setBrakeMode(boolean brakeMode) {
  //   if (brakeMode) {
  //     config.idleMode(IdleMode.kBrake);
  //   } else {
  //     config.idleMode(IdleMode.kCoast);
  //   }
  // }

  @Override
  public double getPosition() {
    return climbMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void setReference(double degrees) {
    climbMotor.getClosedLoopController().setReference(degrees, ControlType.kPosition);
  }

  @Override
  public void applyConfig(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    climbMotor.configure(config, resetMode, persistMode);
  }
}
