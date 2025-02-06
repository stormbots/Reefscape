// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {  
  // private DCMotorSim sim = new DCMotorSim(
  //         LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0, 0),
  //         DCMotor.getNeoVortex(1));

  private SparkFlex sparkFlex = new SparkFlex(9, MotorType.kBrushless);
  private DCMotor dcmotor = DCMotor.getNeoVortex(1);
  private SparkFlexSim sparkSim = new SparkFlexSim(sparkFlex, dcmotor);

  // private LinearSystem plant = LinearSystemId.createSingleJointedArmSystem(dcmotor, 4, 100);
  // private DCMotorSim sim = new DCMotorSim(plant, dcmotor);
  //TODO put this in, as it's a better model as linearysstemID's is terrible.
  private SingleJointedArmSim sim = new SingleJointedArmSim(
    dcmotor, 
    360.0, 
    0.005, 
    0.5, 
    -(2*Math.PI), 
    (2*Math.PI), 
    true, 
    0.0);

  private double appliedVolts = 0.0;
  

  public ClimberIOSim(){
    //crudely attempt to define where we expect the simulation to start
    //Removed because SingleJointedArmSim does this by default
    // var startPosition=100; //degrees
    // sim.setState(Math.toRadians(startPosition), 0);

    //I also don't think this does anything
    //sparkSim.enable();

    // var config = new SparkFlexConfig();
    // config.encoder.positionConversionFactor(1);
    // config.absoluteEncoder.positionConversionFactor(360);

    // config.inverted(true);
    // config
    //     .closedLoop
    //     .outputRange(-0.5, 0.5)
    //     .p(1 / 30.0)
    //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingInputRange(0, 360);
    //     // .positionWrappingInputRange(-180, 180);
    // config.idleMode(IdleMode.kCoast).smartCurrentLimit(5).voltageCompensation(12.0);
    // config.absoluteEncoder.inverted(false);
    // sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // sparkSim = new SparkFlexSim(sparkFlex, dcmotor);
    
    
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var dt = 0.02;
    var vbus = 12;
    inputs.climberVoltage = sparkSim.getAppliedOutput() * vbus;
    //output of the system is now known; We can finally update the simulation
    sim.setInputVoltage(inputs.climberVoltage);

    sim.update(dt);

    var rpm = Units.radiansPerSecondToRotationsPerMinute(sim.getVelocityRadPerSec());
    
    sparkSim.iterate(rpm, vbus, dt);
    sparkSim.getRelativeEncoderSim().setPositionConversionFactor(360);
    
    sparkSim.getAbsoluteEncoderSim().iterate(rpm, dt);
    sparkSim.getRelativeEncoderSim().iterate(rpm, dt);

    //Update the inputs for the log using the system state
    // inputs.climberRelativeAngle = Units.radiansToDegrees(sim.getAngleRads());
    inputs.climberRelativeAngle = sparkSim.getRelativeEncoderSim().getPosition();
    //inputs.climberVoltage = sim.getInputVoltage();
    inputs.climberCurrentDraw = sim.getCurrentDrawAmps();
    inputs.climberAbsoluteAngle = sparkSim.getAbsoluteEncoderSim().getPosition();

    SmartDashboard.putNumber("sim/plantangle", Units.radiansToDegrees(sim.getAngleRads()));

    //TODO: Battery sim
    // climberSim.setInput(sim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    // climberSim.update(0.02);
    // sim.iterate(Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.02);
    // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));
  }

  @Override
  public void setReference(double angle) {
    SmartDashboard.putNumber("sim/reference", angle);

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
}
