// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  // private SparkFlex bruhMotor = new SparkFlex(12, MotorType.kBrushless);
  private SparkFlex pivotMotor = new SparkFlex(13, MotorType.kBrushless);
  private SparkFlex rollerMotor = new SparkFlex(14, MotorType.kBrushless);
  private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
  // private final CoralIntakeIO io;
  //TODO: Find actual values
  private final TrapezoidProfile pivotProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(10, 10));
  private ArmFeedforward pivotFF = new ArmFeedforward(0, 0, 0);

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();


  // CoralIntakeSimulation sim = new CoralIntakeSimulation(pivotMotor,rollerMotor);
  CoralIntakeMech2d mech = new CoralIntakeMech2d();

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    //TODO: Change this
    // io = new CoralIntakeIOReal();
    configPivotMotors();
    configRollerMotors();

    //setDefaultCommand(testRunPivotTrapezoidal(90.0));
  }

  private void configPivotMotors(){
    var config = new SparkFlexConfig();

    config.inverted(true);
    config.smartCurrentLimit(30);

    config.absoluteEncoder
    .positionConversionFactor(360)
    .velocityConversionFactor(360/60)
    .inverted(true)
    ;

    config.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.1/90)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(0, 360)
    
    ;

    
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configRollerMotors(){
    var config = new SparkFlexConfig();

    config.inverted(true);
    config.smartCurrentLimit(40);
    
    var rollerwheeldiameter=2;
    var conversionfactor = Math.PI*rollerwheeldiameter; //convert rotations to inches
    config.encoder
    .positionConversionFactor(conversionfactor)
    .velocityConversionFactor(conversionfactor/60.0); //inches-> InchesPerSecond
    //TODO: Placeholder
    config.closedLoop
    .outputRange(-0.5,0.5)
    .p(0.1/10);

    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // io.updateInputs(inputs);
    // Logger.processInputs("Coral Intake", inputs);
    mech.update(
      getAngle(),
      InchesPerSecond.of(rollerMotor.getEncoder().getVelocity())
      // ,sim.getSimAngle()
    );

    Logger.recordOutput("rollerSpeed", getRollerVelocity().in(InchesPerSecond));
    Logger.recordOutput("pivotAngle", getAngle().in(Units.Degrees));
  }

  @Override
  public void simulationPeriodic() {
    // sim.update();
  }

  ////////////////////////////////////
  /// Commands and public functions
  ////////////////////////////////////



  ////////////////////////////////////
  /// Helper functions
  ////////////////////////////////////
  private double getAdjustedAngle(){
    var angle = pivotMotor.getAbsoluteEncoder().getPosition();
    if(angle > 180) angle -= 360;
    return angle;
  }

  public LinearVelocity getRollerVelocity(){
    return InchesPerSecond.of(rollerMotor.getEncoder().getVelocity());
  }

  public Angle getAngle(){
    return Degrees.of(getAdjustedAngle());
  }

  public Command testRunPivotTrapezoidal(DoubleSupplier angle){
    return startRun(
      ()->{
        pivotSetpoint = new TrapezoidProfile.State(getAdjustedAngle(), pivotMotor.getEncoder().getVelocity());
      }, 
      ()->{
        pivotGoal = new TrapezoidProfile.State(angle.getAsDouble(), 0.0);

        pivotSetpoint = pivotProfile.calculate(0.02, pivotSetpoint, pivotGoal);

        var ff = pivotFF.calculate(pivotSetpoint.position, pivotSetpoint.velocity);

        pivotMotor.getClosedLoopController()
        .setReference(
          pivotSetpoint.position, 
          ControlType.kPosition, 
          ClosedLoopSlot.kSlot0, 
          ff, ArbFFUnits.kVoltage
        );
      }
    );
  }

  public void setRollerVelocity(double velocity){
    rollerMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  public void setPivotAngle(double angle){
    pivotMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public void setAngleSpeed(double angle, double speed){
    setPivotAngle(angle);
    // setRollerVelocity(speed);
    rollerMotor.setVoltage(6);
  }

  public Command intake() {
    return new ParallelCommandGroup(testRunPivotTrapezoidal(()->-45), new RunCommand(()-> setRollerVelocity(2.0)));
  }

  public Command stow() {
    return new ParallelCommandGroup(testRunPivotTrapezoidal(()->90), new RunCommand(()-> setRollerVelocity(0.0)));
  }

  // public Command runStupidEndEffector(){
  //   return run(()->{
  //     bruhMotor.setVoltage(9);
  //   // setRollerVelocity(speed);
  //   rollerMotor.setVoltage(9);
  //   });
  // }

}
