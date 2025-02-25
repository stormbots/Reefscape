// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {  

  private SparkFlex sparkFlex = new SparkFlex(9, MotorType.kBrushless);
  private DCMotor dcmotor = DCMotor.getNeoVortex(1);
  private SparkFlexSim sparkSim = new SparkFlexSim(sparkFlex, dcmotor);
  double startPosition=110; //degrees

  double momentArm=Inches.of(4).in(Meter);
  private SingleJointedArmSim sim = new SingleJointedArmSim(
    dcmotor, 
    360.0, 
    // 0.5*momentArm*momentArm,
    SingleJointedArmSim.estimateMOI(momentArm, 1),
    momentArm, 
    Math.toRadians(-180), 
    Math.toRadians(180), 
    true, 
    Math.toRadians(startPosition)
  ); 

  public ClimberIOSim(){
    sim.setState(Math.toRadians(startPosition), 0);
    sparkSim.setPosition(startPosition);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var dt = 0.02;
    var vbus = RoboRioSim.getVInVoltage();
    
    inputs.climberVoltage = sparkSim.getAppliedOutput() * vbus;

    sim.setInputVoltage(inputs.climberVoltage);
    sim.update(dt);

    var angVel = RadiansPerSecond.of(sim.getVelocityRadPerSec()).in(DegreesPerSecond);
    sparkSim.iterate(angVel, vbus, dt);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    
    inputs.climberAbsoluteAngle = Radians.of(sim.getAngleRads()).in(Degrees);
    inputs.climberRelativeAngle = Radians.of(sim.getAngleRads()).in(Degrees);
    inputs.climberCurrentDraw = sim.getCurrentDrawAmps();
    
    // query the motor approach
    // It may be worth logging *both* in the case that a configuration or other error causes 
    // the sensor angle (which drives the logic) to desync from the sim/plant angle
    // inputs.climberRelativeAngle = getPosition();
    // inputs.climberAbsoluteAngle = getPosition();

  }

  @Override
  public void setReference(double angle) {
    sparkFlex.getClosedLoopController().setReference(angle,ControlType.kPosition);
  }

  @Override
  public void setIdleMode(IdleMode idleMode) {
    var conf = new SparkFlexConfig().idleMode(idleMode);
    configureAsync(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public double getPosition() {
    return sparkSim.getAbsoluteEncoderSim().getPosition();
    // return Units.radiansToDegrees(sim.getAngleRads()); 
  }

  @Override
  public void configureAsync(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    sparkFlex.configure(config, resetMode, persistMode);
  }

  @Override
  public void configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    sparkFlex.configure(config, resetMode, persistMode);
  }

  // @Override
  public void setRelativeEncoderPosition(double position){
    sparkFlex.getEncoder().setPosition(position);
  }

  @Override
  public double getVelocity() {
    return sparkFlex.getAbsoluteEncoder().getVelocity();
  }
}
