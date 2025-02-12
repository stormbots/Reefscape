// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  private SparkFlex pivotMotor = new SparkFlex(22, MotorType.kBrushless);
  private SparkFlex rollerMotor = new SparkFlex(23, MotorType.kBrushless);

  CoralIntakeSimulation sim = new CoralIntakeSimulation(pivotMotor,rollerMotor);
  CoralIntakeMech2d mech = new CoralIntakeMech2d();

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    configPivotMotors();
    configRollerMotors();
  }

  private void configPivotMotors(){
    var config = new SparkFlexConfig();

    config.absoluteEncoder
    .positionConversionFactor(360)
    .velocityConversionFactor(360/60)
    ;

    config.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    ;

    
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configRollerMotors(){
    var config = new SparkFlexConfig();
    
    var rollerwheeldiameter=2;
    var conversionfactor = Math.PI*rollerwheeldiameter; //convert rotations to inches
    config.encoder
    .positionConversionFactor(conversionfactor)
    .velocityConversionFactor(conversionfactor/60.0); //inches-> InchesPerSecond
    
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    mech.update(
      Degrees.of(pivotMotor.getAbsoluteEncoder().getPosition()),
      InchesPerSecond.of(rollerMotor.getEncoder().getVelocity())
      // ,sim.getSimAngle()
    );
  }

  @Override
  public void simulationPeriodic() {
    sim.update();
  }
}
