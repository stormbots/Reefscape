// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private double setpoint=100;

  private final ClimberVisualizer visualizer;
  SlewRateLimiter rateLimit = new SlewRateLimiter(15);
  /** Creates a new Climber. */
  public Climber() {
    SmartDashboard.putBoolean("simulate", false);
    if (RobotBase.isReal()) {
      this.io = new ClimberIOReal();
    }else if(RobotBase.isSimulation()){
      this.io = new ClimberIOSim();
    } else {
      this.io = new ClimberIO() {}; 
      //TODO: Make it so replay actually works, as we have no functional mode switch right now
      // Create a blank stub for replay
    }

  // 0 deg, 50.711
  // 88 deg, 153.705
  // lower softlimit 300 deg (abs) around inside bot
  // upper softlimit/stow 43 (abs) deg
  // motor invert true, absolute encoder fine

    io.configure(getClimberMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Do some final tidying up. 
    io.setRelativeEncoderPosition(io.getPosition());
    isOnTarget = new Trigger(()->MathUtil.isNear(setpoint, getPosition(), 3)).debounce(0.1);
    rateLimit.reset(getPosition());

    visualizer = new ClimberVisualizer("climber");

    //TODO: Remove automated motion until tested
    SmartDashboard.putData("subsystems/climber",this);

    setDefaultCommand(holdPosition());
  }


  private SparkFlexConfig getClimberMotorConfig(){
    var config = new SparkFlexConfig();
    config = new SparkFlexConfig();
    //unused and uncalibrated value config.encoder.positionConversionFactor((360 / (153.705 - 50.711 / 88.0)));
    config.encoder
    .positionConversionFactor(1);
    config.absoluteEncoder
    .positionConversionFactor(360)
    .velocityConversionFactor(360/60);

    config.inverted(false);
    config.closedLoop
        .outputRange(-0.5, 0.5)
        .p(0.7 / 30.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 360);

    config
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(60)
    .voltageCompensation(11.0);

    config.absoluteEncoder.inverted(true);
    config.softLimit
        .forwardSoftLimit(43)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(-60)
        .reverseSoftLimitEnabled(false);

    return config;
  }


  @Override
  public void periodic() {
    //update and log inputs
    //TODO: We do not know the inputs to update yet, because 
    //they're generated by motor controller: Which is an IO/Real/Sim distinction.
    //We need to read them from the sim, and *then* have it log them.
    //Which makes this wrong
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    visualizer.update(inputs.climberAbsoluteAngle);
    SmartDashboard.putNumber("climber/ClimberAngle", getPosition());
    SmartDashboard.putNumber("climber/Setpoint", setpoint);
  }

  public Command prepareToClimb() {
    return new InstantCommand(()->setIdleMode(IdleMode.kCoast))
    .andThen(setAngle(()->-60));
  }

  public Command stow() {
    return new InstantCommand(()->setIdleMode(IdleMode.kCoast))
    .andThen(setAngle(()->110));
  }

  public Command climb() {
    return new InstantCommand(()->setIdleMode(IdleMode.kBrake))
    .andThen(setAngle(()->0))
    .andThen(setAngle(()->-30))
    .andThen(setAngle(()->5));
  }
  
  public Command setAngle(DoubleSupplier angle) {
    return startRun(
      ()-> rateLimit.reset(getPosition()),
      () -> setPosition(MathUtil.clamp(angle.getAsDouble(), -90, 45))
      ).until(isOnTarget)
      ;
  }

  public Trigger isOnTarget;// Must be initialized in constructor after IO

  private void setPosition(double angle) {
    setpoint=angle;
    io.setReference(rateLimit.calculate(setpoint));
  }

  public void setIdleMode(IdleMode idleMode) {
    var config = new SparkFlexConfig().idleMode(idleMode);
    io.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Go to the setpoint if we're actually near it, otherwise just halt 
   * Without doing this, you wind up being off by the allowed tolerance */
  public Command holdPosition(){
    return new ConditionalCommand(
      setAngle(()->setpoint),
      startRun(
        //Set the position to where we're at now, ignoring potential rate limiting
        () -> {setpoint = getPosition(); io.setReference(setpoint);}, 
        ()-> {} // Do nothing for the lifetime of the command
      ),
      isOnTarget::getAsBoolean
    );
  }

  public Command stop(){
    return run(()->io.stop());
  }


  private double getPosition(){
    var angle = io.getPosition();
    if(angle>180) angle = angle-360;
    return angle;
  }

  public Angle getAngle(){
    return Degrees.of(getPosition());
  }

}
