// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  public boolean isHomed = false;
  public double power;
  public int current;
  private final double toleranceHeight = 2;
  private final double toleranceAngle = 5;
  private final double kArmMaxVelocity = 10.0;
  private final double kArmMaxAcceleration = 2.0;
  private final TrapezoidProfile armTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kArmMaxVelocity, kArmMaxAcceleration));
  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State(); 
  ElevatorPose setpoint = new ElevatorPose(0, 0, 0);

  SparkFlex elevatorMotor = new SparkFlex(10, MotorType.kBrushless);
  SparkFlex rotationMotor = new SparkFlex(11, MotorType.kBrushless);
  SparkFlex coralOutMotor = new SparkFlex(12, MotorType.kBrushless);

  ElevatorMech2d mechanism = new ElevatorMech2d();
  Optional<ElevatorSimulation> sim = Robot.isSimulation()?Optional.of(new ElevatorSimulation(elevatorMotor, rotationMotor, coralOutMotor)):Optional.empty();

  public static final Angle ElevatorArmMinSoftLimitMin=Degrees.of(-75);
  public static final Angle ElevatorArmMinSoftLimitMax=Degrees.of(135);

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



  public final ElevatorPose kStationPickup =  new ElevatorPose(5, 60, -10);
  //need special procedure to move to floor pickup
  private final ElevatorPose kFloorPickup =    new ElevatorPose(28.5, -75, -10);
  public final ElevatorPose kPrepareToFloorPickup = new ElevatorPose(44.5, -75, 0);
  public final ElevatorPose kStowed =         new ElevatorPose(0, 84, 0);
  public final ElevatorPose kStowedUp =         new ElevatorPose(44.5, 84, 0);
  public final ElevatorPose kStowedUpBruh =         new ElevatorPose(45, 84, 0);
  public final ElevatorPose kClimbing =       new ElevatorPose(0, 90, 0);
  public final ElevatorPose kL1 =             new ElevatorPose(24, 90, 10);
  public final ElevatorPose kL2 =             new ElevatorPose(30, 135, 10);
  public final ElevatorPose kL3 =             new ElevatorPose(36, 135, 10);
  public final ElevatorPose kL4 =             new ElevatorPose(60, 145, 10);
 // public final ElevatorPose kL4bruh =             new ElevatorPose(60, 90, 0);


  SparkBaseConfig elevatorHighPowerConfig = new SparkMaxConfig().smartCurrentLimit(40);

  ArmFeedforward rotatorFF = new ArmFeedforward(0.3, 0.25, 0.0, 0.0);
  ElevatorFeedforward elevatorFF = new ElevatorFeedforward((.45 -(-.071))/2, (.45 -.071)/2, 0.0);
  public Elevator() {

    var startingheight = getCarriageHeight().in(Inches);

    //Set up motor configs
    elevatorMotor.configure(ElevatorMotorConfigs.getElevatorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralOutMotor.configure(ElevatorMotorConfigs.getScorerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotationMotor.configure(ElevatorMotorConfigs.getRotationConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(0.02);
    elevatorMotor.getEncoder().setPosition(startingheight);

    double absoluteAngle = rotationMotor.getAbsoluteEncoder().getPosition();
    if(absoluteAngle>225){
      absoluteAngle-=360;
    }
    //should probably do again, sometimes doesnt get sent
    rotationMotor.getEncoder().setPosition(absoluteAngle);
    // new Trigger(DriverStation::isEnabled).and(() -> isHomed == false).onTrue(homeElevator());
    new Trigger(DriverStation::isDisabled).onTrue(runOnce(()->rotationMotor.stopMotor()));

    // setDefaultCommand(holdPosition());
    setDefaultCommand(run(()->{
      elevatorMotor.setVoltage(elevatorFF.getKg());
      rotationMotor.setVoltage(Math.cos(getRelativeArmAngle().in(Radians))*rotatorFF.getKg());
      coralOutMotor.stopMotor();
    }));
  }
  
  public Trigger haveCoral = new Trigger( () -> {
    return elevatorMotor.getOutputCurrent() > current;
  }).debounce(0.1);

  public Trigger isAtSafePosition = new Trigger( () ->  getCarriageHeight().in(Inch) > kStowedUp.height-1 )
  .and( ()-> getRelativeArmAngle().in(Degrees) > 0 && getRelativeArmAngle().in(Degrees) < 93);


  public Command homeElevator() {
    // step 0: Make sure scorer is in a safe position

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

  private Angle getRelativeArmAngle(){
    return Degrees.of(rotationMotor.getEncoder().getPosition());
  }

  private Distance getCarriageHeight(){
    return Inches.of(elevatorMotor.getEncoder().getPosition());
  }

  private void setHeight(double height) {
    var ff = elevatorFF.getKg();
    elevatorMotor
        .getClosedLoopController()
        .setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    setpoint.height = height;
  }

  SlewRateLimiter slewRateAngle = new SlewRateLimiter(75);
  
  private void setAngle(double angle) {
    // angle = MathUtil.clamp(angle, ElevatorArmMinSoftLimitMin.in(Degrees), ElevatorArmMinSoftLimitMax.in(Degrees));
    //if setpoint is not set before slewrate, isAtSetpoint trigger will activate instantly
    setpoint.angle = angle;
    angle = slewRateAngle.calculate(angle);
    SmartDashboard.putNumber("elevator/slewRatedAngle", angle);
  
    // var ff = rotatorFF.calculate(getRelativeArmAngle().in(Radian), 0);
    var ff = Math.cos(getRelativeArmAngle().in(Radians))*rotatorFF.getKg();
    rotationMotor
      .getClosedLoopController()
      .setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
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
        armSetpoint = new TrapezoidProfile.State(getRelativeArmAngle().in(Degrees), rotationMotor.getEncoder().getVelocity());
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
  private Command moveToHeight(double height) {
    return runEnd(
      () -> setHeight(height),
      () -> {}// elevatorMotor.set(0)
    ).until(atTargetHeight);
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
  
  // public Command moveToAngle(double angle) {
  //   return startRun(
  //       () -> {
  //         slewRateAngle.reset(getRelativeArmAngle().in(Degrees));
  //       },
  //       () -> {
  //         setAngle(angle);
  //       }
  //     // exit condition on arriving?
  //   ).until(atTargetAngle);
  // }

  
  private Command moveToPose(ElevatorPose pose) {
    return startRun(
        () -> {
          slewRateAngle.reset(getRelativeArmAngle().in(Degrees));
        },
        () -> {
          setHeight(pose.height);
          setAngle(pose.angle);
          setScorerSpeed(0);
        }
    // exit condition on arriving?
    );
  }

  public Command moveToPoseSafe(ElevatorPose pose) {
    return new SequentialCommandGroup(
      moveToPose(kStowedUp).until(isAtSafePosition),
      moveToPose(pose)
    );
  }
  public Command scoreAtPoseSafe(ElevatorPose pose) {
    return moveToPoseSafe(pose)
    .andThen(moveToPoseWithScorer(pose));
  }


  public Command goToIntake(Trigger isDown){
    return new SequentialCommandGroup(
      moveToPoseSafe(kPrepareToFloorPickup),
      Commands.idle(this).until(isDown),
      moveToPose(kFloorPickup)
    );
  }


  //Move to prepare to floor pickup, wait for intake to lower(also cuz if we rotate straight to intake we ram into support)
  public Command moveToIntake(Trigger lowerIntake){
    return new SequentialCommandGroup(
      moveToPoseSafe(kPrepareToFloorPickup),
      new WaitCommand(999999).until(lowerIntake),
      moveToHeight(kFloorPickup.height)
    );
    //In theory, move back up at very end. doesnt work, causes things to break, whatever
    // .finallyDo((e)->moveToHeight(kPrepareToFloorPickup.height));
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
          slewRateAngle.reset(getRelativeArmAngle().in(Degrees));
        },
        () -> {
          setHeight(pose.height);
          setAngle(pose.angle);
          setScorerSpeed(pose.speed);
        }
    // exit condition on arriving?
    );
  }

  public Command holdPosition(){
    return run(
      () -> {
        setHeight(getCarriageHeight().in(Inches));
        setAngle(getRelativeArmAngle().in(Degrees));
        setScorerSpeed(coralOutMotor.getEncoder().getPosition());
      }
    );
  }


  public Trigger atTargetPosition =
    new Trigger(() -> {
      return 
      MathUtil.isNear(setpoint.height, getCarriageHeight().in(Inches), toleranceHeight)
      && 
      MathUtil.isNear(setpoint.angle, getRelativeArmAngle().in(Degrees), toleranceAngle);
    }).debounce(0.02*3)
    ;

  public Trigger atTargetAngle = 
    new Trigger(()->{
      return MathUtil.isNear(setpoint.angle, getRelativeArmAngle().in(Degrees), toleranceAngle);
    })
  ;

  public Trigger atTargetHeight = 
    new Trigger(()->{
      return MathUtil.isNear(setpoint.height, getCarriageHeight().in(Inches), toleranceHeight);
    })
  ;

  public boolean bounded(double value, double min, double max) {
    return (value >= min && value <= max);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("elevator/height", getCarriageHeight().in(Inches));
    SmartDashboard.putNumber("elevator/setpoint", setpoint.height);
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
      getCarriageHeight().in(Inches),
      getRelativeArmAngle().in(Degree),
      coralOutMotor.getEncoder().getVelocity()
    );
  }

  @Override
  public void simulationPeriodic() {
    if(sim.isEmpty())return;
    sim.get().update();;
    // SmartDashboard.putNumber("elevaor/angle/sim angle", sim.getAngle().in(Degrees))
  }


  public Command setVoltage(DoubleSupplier voltage){
    return run(()->{
      SmartDashboard.putNumber("elevator/voltage", voltage.getAsDouble());
      elevatorMotor.setVoltage(elevatorFF.calculate(0));
      rotationMotor.setVoltage(voltage.getAsDouble());
    }).finallyDo(()->{
      elevatorMotor.setVoltage(0);
      rotationMotor.setVoltage(0);
    });
  }
}
