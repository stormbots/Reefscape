// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.stormbots.LaserCanWrapper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class CoralIntake extends SubsystemBase {

  // private SparkFlex bruhMotor = new SparkFlex(12, MotorType.kBrushless);
  private SparkFlex pivotMotor = new SparkFlex(13, MotorType.kBrushless);
  private SparkFlex rollerMotor = new SparkFlex(14, MotorType.kBrushless);
  // private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
  // private final CoralIntakeIO io;
  //TODO: Find actual values
  private final TrapezoidProfile pivotProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(90/0.05, 270));
  private ArmFeedforward pivotFF = new ArmFeedforward(0, 0, 0);
  public LaserCanWrapper laserCan = new LaserCanWrapper(24)
    .configureShortRange()
    .setThreshhold(Inches.of(2))
    ;

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();


  Optional<CoralIntakeSimulation> sim = Robot.isSimulation()?Optional.of(new CoralIntakeSimulation(pivotMotor,rollerMotor)):Optional.empty();
  CoralIntakeMech2d mech = new CoralIntakeMech2d();

  /** Creates a new CoralIntake. */
  public CoralIntake(Trigger elevatorIsClear) {
    //TODO: Change this
    // io = new CoralIntakeIOReal();
    configPivotMotors();
    configRollerMotors();

    setDefaultCommand(stow(elevatorIsClear));
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
    .p(0.5/90)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(0, 360)
    
    ;

    
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  private void configRollerMotors(){
    var config = new SparkFlexConfig();

    config.inverted(true);
    config.smartCurrentLimit(50);//60 will def cause slip, 40 is not enough
    
    // var rollerwheeldiameter=2; //TODO no clue
    var conversionfactor =1;// Math.PI*rollerwheeldiameter; //convert rotations to inches
    config.encoder
    .positionConversionFactor(conversionfactor)
    .velocityConversionFactor(conversionfactor); //inches-> InchesPerSecond
    //TODO: Placeholder
    config.closedLoop
    .outputRange(-0.5,0.5)
    .velocityFF(1/5784.0*conversionfactor)
    // .p(0.5/25);
    .p(0);

    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Coral Intake", inputs);
    mech.update(
      getAngle(),
      getRollerVelocity()
      // ,sim.getSimAngle()
    );

    // Logger.recordOutput("rollerSpeed", getRollerVelocity().in(InchesPerSecond));
    // Logger.recordOutput("pivotAngle", getAngle().in(Units.Degrees));
  }

  @Override
  public void simulationPeriodic() {
    if(sim.isEmpty())return;
    sim.get().update();
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

  public Command setAngle(DoubleSupplier angle){
    return startRun(
      ()->{
        pivotSetpoint = new TrapezoidProfile.State(getAdjustedAngle(), pivotMotor.getAbsoluteEncoder().getVelocity());
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

  private void setRollerVelocity(double velocity){
    rollerMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  /** Should not be used outside special cases */
  private void setPivotAnglePID(double angle){
    pivotMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public Command setAngleSpeed(DoubleSupplier angle, DoubleSupplier velocity ){
    return new ParallelCommandGroup(
      setAngle(angle),
      //TODO fix pids D:
      new RunCommand(()-> setRollerVelocity(velocity.getAsDouble()))
      // new RunCommand(()->rollerMotor.setVoltage(6))// TEMP TO MAKE ROBOT GO
    );
  }

  public Command intake(Trigger hasCoral) {
    Timer timer = new Timer();
    return new SequentialCommandGroup(
      new InstantCommand(()->timer.reset()), //weird bug fix
      setAngleSpeed(()->-46, ()->4500).until(hasCoral.and(()->timer.hasElapsed(0.5)))
    );
    // return setAngleSpeed(()->-45, ()->25).until(hasCoral);
  }

  public Command stow(BooleanSupplier elevatorClear) {
    // return new WaitCommand(99999).until(elevatorClear).andThen(setAngleSpeed(()->55, ()->0));
    Command stow = new SequentialCommandGroup(
      new InstantCommand(()->rollerMotor.set(0)),
      new WaitCommand(99999).until(elevatorClear),
      setAngleSpeed(()->55, ()->0)//.onlyWhile(elevatorClear)
    ).repeatedly();

    stow.addRequirements(this);
    return stow;
  }

  public Trigger readyToClimb = new Trigger(()->true); //TODO: Do we have conditions for this?
  public Trigger readyToLoad = new Trigger(()->getAdjustedAngle()<30);
  public Trigger stowed = new Trigger(()->getAdjustedAngle()>70);
  public Trigger coralLoaded = laserCan.isBreakBeamTripped.debounce(0.03);
 

  public Trigger isOnTarget = new Trigger(()->{
    return MathUtil.isNear(pivotGoal.position, getAngle().in(Degrees), 5);
  });
}
