// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
  public boolean isHomed = false;
  public double power;
  public int current;
  private final double toleranceHeight = 0.5;
  private final double toleranceAngle = 5;
  private final double kArmMaxVelocity = 10.0;
  private final double kArmMaxAcceleration = 2.0;
  private final TrapezoidProfile armTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kArmMaxVelocity, kArmMaxAcceleration));
  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State(); 
  ElevatorPose setpoint = new ElevatorPose(0, 0, 0);

  SparkFlex elevatorMotor = new SparkFlex(5, MotorType.kBrushless);
  SparkFlex rotationMotor = new SparkFlex(12, MotorType.kBrushless);
  SparkFlex coralOutMotor = new SparkFlex(13, MotorType.kBrushless);
  SparkFlex elevatorMotorFollower = new SparkFlex(11, MotorType.kBrushless);

  ElevatorMech2d mechanism = new ElevatorMech2d();
  ElevatorSimulation sim = new ElevatorSimulation(elevatorMotor, rotationMotor, coralOutMotor);

  public static final Angle ElevatorArmMinSoftLimitMin=Degrees.of(30);
  public static final Angle ElevatorArmMinSoftLimitMax=Degrees.of(90+30);

  public class ElevatorPose {
    double height;
    double angle;
    double speed;

    public ElevatorPose(double height, double angle, double speed) {
      this.height = height;
      this.angle = angle;
      this.speed = speed;
    }
  }


  //These values are  not tuned


  public final ElevatorPose kMoveToSourcePickUp = new ElevatorPose(5, 60, 0);
  public final ElevatorPose kSourcePickup = new ElevatorPose(5, 60, -10);
  public final ElevatorPose kMoveToFloorPickUp = new ElevatorPose(0, 15, 0);
  public final ElevatorPose kFloorPickup = new ElevatorPose(0, 15, -10);
  public final ElevatorPose kStowed = new ElevatorPose(0, 90, 0);
  public final ElevatorPose kStowedUp = new ElevatorPose(20, 90, 0);
  public final ElevatorPose kClimbing = new ElevatorPose(0, 90, 0);
  public final ElevatorPose kMoveToL1Algae = new ElevatorPose(27, 90, 0);
  public final ElevatorPose kMoveToL2Algae = new ElevatorPose(33, 90, 0);
  public final ElevatorPose kIntakeAlgaeL1 = new ElevatorPose(27, 90, 10);
  public final ElevatorPose kIntakeAlgaeL2 = new ElevatorPose(33,90, 10);
  public final ElevatorPose kL1 = new ElevatorPose(24, 90, 10);
  public final ElevatorPose kL2 = new ElevatorPose(30, 135, 10);
  public final ElevatorPose kL3 = new ElevatorPose(36, 135, 10);
  public final ElevatorPose kL4 = new ElevatorPose(40, 135, 10);

  SparkBaseConfig elevatorHighPowerConfig = new SparkMaxConfig().smartCurrentLimit(40);

  ArmFeedforward rotatorFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.0, 0.0, 0.0);
  public Elevator() {

    //Set up motor configs
    elevatorMotor.configure(ElevatorMotorConfigs.getElevatorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralOutMotor.configure(ElevatorMotorConfigs.getScorerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotationMotor.configure(ElevatorMotorConfigs.getRotationConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //Practice bot only!!!
    var elevatorFollowerConfig = new SparkFlexConfig()
      .apply(ElevatorMotorConfigs.getElevatorConfig())
      .follow(elevatorMotor,true);
    elevatorMotorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rotationMotor.getEncoder().setPosition(rotationMotor.getAbsoluteEncoder().getPosition());
    // new Trigger(DriverStation::isEnabled).and(() -> isHomed == false).onTrue(homeElevator());
    new Trigger(DriverStation::isDisabled).onTrue(runOnce(()->rotationMotor.stopMotor()));
  }

  public Trigger haveCoral = new Trigger( () -> {
    return elevatorMotor.getOutputCurrent() > current;
  }).debounce(0.1);

  public Command homeElevator() {
    // step 0: Make sure scorerisin safeposition

    return run(() -> {
          // logic here
          // run elevators  down
          if (isHomed == false) {
            elevatorMotor.set(power);
          }
          if (elevatorMotor.getOutputCurrent() > current) {
            elevatorMotor.set(0);
            isHomed = true;
          }
        })
        .finallyDo(
            (wasInterrupted) -> {
              if (wasInterrupted) {
              } else {
                // homed correclty
                // set power
                // sethomedflag
              }
            });
  }

  private Angle getArmAngle(){
    return Degrees.of(rotationMotor.getAbsoluteEncoder().getPosition());
  }

  private Distance getCarraigeHeight(){
    return Inches.of(elevatorMotor.getAbsoluteEncoder().getPosition());
  }

  private void setHeight(double height) {
    var ff = elevatorFF.getKg();
    elevatorMotor
        .getClosedLoopController()
        .setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    setpoint.height = height;
  }

  SlewRateLimiter slewRateAngle = new SlewRateLimiter(2);
  
  private void setAngle(double angle) {
    angle = MathUtil.clamp(angle, ElevatorArmMinSoftLimitMin.in(Degrees), ElevatorArmMinSoftLimitMax.in(Degrees));
    angle = slewRateAngle.calculate(angle);
  
    //var ff = rotatorFF.calculate(getArmAngle().in(Radian), 0);
    var ff = 0;
    rotationMotor
      .getClosedLoopController()
      .setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    setpoint.angle = angle;
  }

  private void setScorerSpeed(double speed) {
    coralOutMotor
        .getClosedLoopController()
        .setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
    setpoint.speed = speed;
  }
  

  public Command testMoveElevatorArmWithTrap(DoubleSupplier position){
    return startRun(
      ()->{
        //Seed the initial state/setpoint with the current state
        armSetpoint = new TrapezoidProfile.State(getArmAngle().in(Degrees), rotationMotor.getAbsoluteEncoder().getVelocity());
      }, 
      ()->{
        //Make sure the goal is dynamically updated
        armGoal = new TrapezoidProfile.State(position.getAsDouble(), 0);

        //update our setpoint to be our next state
        armSetpoint = armTrapezoidProfile.calculate(0.02, armSetpoint, armGoal);
    
        //var ff = rotatorFF.calculate(armSetpoint.position, armSetpoint.velocity);
        var ff = 0;
        rotationMotor.getClosedLoopController()
        .setReference(
          armSetpoint.position,
          ControlType.kPosition, ClosedLoopSlot.kSlot0,
          ff, ArbFFUnits.kVoltage
        );
      }
    );
  }
  public Command moveToHeight(double height) {
    return runEnd(
      () -> setHeight(height),
      () -> {}// elevatorMotor.set(0)
    );
  }

  public Command manualElevatorPower(double power) {
    return runEnd(
      () -> {
        elevatorMotor.setVoltage(12*power+elevatorFF.getKg());
        SmartDashboard.putNumber("powerIn", (power));
      },
      () -> {
        elevatorMotor.set(0);
      } 
    );
  }
  
  public Command moveToAngle(double angle) {
    return startRun(
        () -> {
          slewRateAngle.reset(getArmAngle().in(Degrees));
        },
        () -> {
          setAngle(angle);
        }
      // exit condition on arriving?
    );
  }

  public Command moveToPose(ElevatorPose pose) {
    return startRun(
        () -> {
          slewRateAngle.reset(getArmAngle().in(Degrees));
        },
        () -> {
          setHeight(pose.height);
          setAngle(pose.angle);
          setScorerSpeed(0);
        }
    // exit condition on arriving?
    );
  }

  SysIdRoutineLog log = new SysIdRoutineLog("elevatorArm");
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(this.rotationMotor::setVoltage, (logg)->{}, this)
  );

  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command moveToPoseWithScorer(ElevatorPose pose) {
    return startRun(
        () -> {
          slewRateAngle.reset(getArmAngle().in(Degrees));
        },
        () -> {
          setHeight(pose.height);
          setAngle(pose.angle);
          setScorerSpeed(pose.speed);
        }
    // exit condition on arriving?
    );
  }

  public Command scoreAtPose(ElevatorPose pose) {
    return moveToPose(pose)
        .until(atTargetPosition /* .and(chassis.atproperreefposition) */)
        .andThen(moveToPoseWithScorer(pose));
  }


  public Trigger atTargetPosition =
    new Trigger(() -> {
      return 
      MathUtil.isNear(setpoint.height, elevatorMotor.getEncoder().getPosition(), toleranceHeight)
      && 
      MathUtil.isNear(setpoint.angle, rotationMotor.getAbsoluteEncoder().getPosition(), toleranceAngle);
    }).debounce(0.02*3)
    ;

  public boolean bounded(double value, double min, double max) {
    return (value >= min && value <= max);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("elevator/height", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator/appliedvoltage", elevatorMotor.getAppliedOutput()*rotationMotor.getBusVoltage());
    SmartDashboard.putNumber("elevator/current", elevatorMotor.getOutputCurrent());
    
    
    SmartDashboard.putNumber("elevator/out-motor/position", coralOutMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator/out-motor/voltage", coralOutMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber("elevator/rotation/abs", rotationMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("elevator/rotation/rel", rotationMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("elevator/angle/Current", rotationMotor.getOutputCurrent());
    SmartDashboard.putNumber("elevator/angleVoltage",rotationMotor.getAppliedOutput()*rotationMotor.getBusVoltage());
    SmartDashboard.putNumber("elevator/inputVoltage",rotationMotor.getBusVoltage());
    
    mechanism.update(
      //TODO: Make this intake a CoralInputs object and read from that
      elevatorMotor.getEncoder().getPosition(),
      rotationMotor.getAbsoluteEncoder().getPosition(),
      coralOutMotor.getEncoder().getVelocity()
    );
  }

  @Override
  public void simulationPeriodic() {
      sim.update();
      SmartDashboard.putNumber("elevaor/angle/sim angle", sim.getAngle().in(Degrees));
  }

}
