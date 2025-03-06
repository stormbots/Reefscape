// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.BooleanSupplier;
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
import com.stormbots.LaserCanWrapper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  public boolean isHomed = false;
  public int ROLLERSTALLCURRENT;
  private final double toleranceHeight = 1;
  private final double toleranceHeightUnfolding = 0.5;
  private final double toleranceAngle = 5;
  private final double kArmMaxVelocity = 180.0;
  private final double kArmMaxAcceleration = 270.0;
  private final TrapezoidProfile armTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kArmMaxVelocity, kArmMaxAcceleration));
  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State(); 

  /** System Goal State */
  ElevatorPose setpoint = new ElevatorPose(0, 0, 0);
  /** This is the interrim setpoint used by the trapezoidal profile */
  ElevatorPose setpointIntermediate = new ElevatorPose(0, 0, 0);

  boolean hasArmSynced = false;

  SparkFlex elevatorMotor = new SparkFlex(10, MotorType.kBrushless);
  SparkFlex rotationMotor = new SparkFlex(11, MotorType.kBrushless);
  SparkFlex coralOutMotor = new SparkFlex(12, MotorType.kBrushless);

  private final Distance kNoCoralDistance = Millimeter.of(50);
  LaserCanWrapper laserCan = new LaserCanWrapper(22)
    .configureShortRange()
    .setThreshhold(kNoCoralDistance)
    ;


  ElevatorMech2d mechanism = new ElevatorMech2d();
  Optional<ElevatorSimulation> sim = Robot.isSimulation()?Optional.of(new ElevatorSimulation(elevatorMotor, rotationMotor, coralOutMotor)):Optional.empty();

  public static final Angle ElevatorArmMinSoftLimitMin=Degrees.of(-75);
  public static final Angle ElevatorArmMinSoftLimitMax=Degrees.of(180);

  

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


  public final ElevatorPose kStationPickup =  new ElevatorPose(13.2, 57, 2500);
  //need special procedure to move to floor pickup
  private final ElevatorPose kFloorPickup =    new ElevatorPose(26.3, -66, -10);
  public final ElevatorPose kPrepareToFloorPickup = new ElevatorPose(43, -66, 0);
  public final ElevatorPose kStowed =         new ElevatorPose(0, 84, 0);
  public final ElevatorPose kStowedUp =         new ElevatorPose(26, 84, 0);
  public final ElevatorPose kClimbing =       new ElevatorPose(16, 128, 0);
  public final ElevatorPose kL1 =             new ElevatorPose(24, 90, 10);
  public final ElevatorPose kL2 =             new ElevatorPose(21, 145.5, 10);
  public final ElevatorPose kL3 =             new ElevatorPose(37, 145.5, 10);
  public final ElevatorPose kL4 =             new ElevatorPose(59.4, 135, 10);
  public final ElevatorPose kL2Algae =             new ElevatorPose(23.8, 142, -2500);
  public final ElevatorPose kL2AlgaeFar =             new ElevatorPose(25.0, 152, -2500);
  public final ElevatorPose kL3Algae =             new ElevatorPose(39.5, 142, -2500);



  SparkBaseConfig elevatorHighPowerConfig = new SparkMaxConfig().smartCurrentLimit(40);

  // ArmFeedforward rotatorFF = new ArmFeedforward(0.1, 0.25, 0.0, 0.0);
  ArmFeedforward rotatorFF = new ArmFeedforward(0.0, 0.25, 0.0, 0.0);
  ElevatorFeedforward elevatorFF = new ElevatorFeedforward((.45 -(-.071))/2, (.45 -.071)/2, 0.0);
  public Elevator() {

    var startingheight = getCarriageHeight().in(Inches);

    //Set up motor configs
    elevatorMotor.configure(ElevatorMotorConfigs.getElevatorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralOutMotor.configure(ElevatorMotorConfigs.getScorerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotationMotor.configure(ElevatorMotorConfigs.getRotationConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(0.1); //Wait until motors are happy and providing us correct data

    elevatorMotor.getEncoder().setPosition(startingheight);

    //should probably do again, sometimes doesnt get sent
    rotationMotor.getEncoder().setPosition(getAngleAbsolute().in(Degrees));
    // new Trigger(DriverStation::isEnabled).and(() -> isHomed == false).onTrue(homeElevator());
    new Trigger(DriverStation::isEnabled).and(()->!hasArmSynced)
    .onTrue(new InstantCommand(()->{
      rotationMotor.getEncoder().setPosition(getAngleAbsolute().in(Degrees));
      hasArmSynced=true;
    }));
    new Trigger(DriverStation::isDisabled).onTrue(runOnce(()->rotationMotor.stopMotor()));

    // setDefaultCommand(holdPosition());
    setDefaultCommand(run(()->{
      elevatorMotor.setVoltage(elevatorFF.calculate(0));
      rotationMotor.setVoltage(rotatorFF.calculate(getAngle().in(Degree), 0));
      coralOutMotor.stopMotor();
    }));

  }

  public Trigger isCoralInScorer = laserCan.isBreakBeamTripped;

  public Trigger isCoralScorerStalled = new Trigger( () -> {
    return coralOutMotor.getOutputCurrent() > ROLLERSTALLCURRENT;
  }).debounce(0.1);

  public Trigger isAtSafePosition = new Trigger( () ->  getCarriageHeight().in(Inch) > kStowedUp.height-1 )
  .and( ()-> getAngle().in(Degrees) > 0 && getAngle().in(Degrees) < 93);


  public Command homeElevator() {
    // step 0: Make sure scorer is in a safe position

     return 
     moveToAngleTrap(()->84.0).until(isAtTargetPosition)
    .andThen(
      run(() -> {
            elevatorMotor.set(-0.1);
      })
      .finallyDo( (wasInterrupted) -> {
        if (wasInterrupted) {
        } else {
          elevatorMotor.stopMotor();
          elevatorMotor.getEncoder().setPosition(0);
          // isHomed = true;
        }
      })
      .until(()->isHomed || elevatorMotor.getOutputCurrent() > 4)
    );
  }

  private Angle getAngleAbsolute(){
    //TODO: Make this have rotation handling
    double absoluteAngle = rotationMotor.getAbsoluteEncoder().getPosition();
    if(absoluteAngle>225){
      absoluteAngle-=360;
    }
    return Degrees.of(absoluteAngle);
  }

  public Angle getAngle(){
    return Degrees.of(rotationMotor.getEncoder().getPosition());
  }

  public AngularVelocity getAngularVelocity(){
    return DegreesPerSecond.of(rotationMotor.getEncoder().getVelocity());
  }

  public Distance getCarriageHeight(){
    return Inches.of(elevatorMotor.getEncoder().getPosition());
  }

  private void setHeight(double height) {
    var ff = elevatorFF.getKg();
    elevatorMotor
        .getClosedLoopController()
        .setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    setpoint.height = height;
  }

  SlewRateLimiter slewRateAngle = new SlewRateLimiter(60);
  
  private void setAngle(double angle) {
    angle = MathUtil.clamp(angle, ElevatorArmMinSoftLimitMin.in(Degrees), ElevatorArmMinSoftLimitMax.in(Degrees));
    setpoint.angle = angle;
    angle = slewRateAngle.calculate(angle);
    SmartDashboard.putNumber("elevator/slewRatedAngle", angle);
  
    var ff = rotatorFF.calculate(getAngle().in(Radians), 0);
    rotationMotor
      .getClosedLoopController()
      .setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
  }

  private void setScorerSpeed(double speed) {
    coralOutMotor
        .getClosedLoopController()
        .setReference(speed, ControlType.kVelocity);
    // , ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
    // coralOutMotor.setVoltage(SmartDashboard.getNumber("elevator/scorerSpeed", 0.000000000001)); //Temporary, just to make coral pickup work..., bruh
    setpoint.speed = speed;
  }
  

  //TODO: The phrasing and management of this is all awful;
  // This has "setpoint" being used as an intermediary, whereas the rest of the system uses it 
  // as the "goal" state.
  // They're also being set independently and checked independently, which *will* result in bugs and jank
  // We will need to fix this before Salem. 
  Trigger isProfileMotionComplete = new Trigger(()->{
    return MathUtil.isNear(armSetpoint.position, armGoal.position, toleranceAngle) 
    && MathUtil.isNear(armSetpoint.velocity, armGoal.velocity, 20 /*degrees per second*/);
  });

  public Command moveToAngleTrap(DoubleSupplier position){
    return startRun(
      ()->{
        //Seed the initial state/setpoint with the current state
        setpoint.angle=position.getAsDouble();
        armSetpoint = new TrapezoidProfile.State(getAngle().in(Degrees), rotationMotor.getEncoder().getVelocity());
      }, 
      ()->{
        //Make sure the goal is dynamically updated
        armGoal = new TrapezoidProfile.State(position.getAsDouble(), 0);

        //update our setpoint to be our next state
        armSetpoint = armTrapezoidProfile.calculate(0.02, armSetpoint, armGoal);
    
        var ff = rotatorFF.calculate(armSetpoint.position, armSetpoint.velocity);
        rotationMotor.getClosedLoopController()
        .setReference(
          armSetpoint.position,
          ControlType.kPosition, ClosedLoopSlot.kSlot0,
          ff, ArbFFUnits.kVoltage
        );
      }
    )
    // .until(isProfileMotionComplete)
    ;
  }

  //
  public Command runCoralScorer(double speed){
    return new RunCommand(()->setScorerSpeed(speed));
    // return new RunCommand(()->coralOutMotor.setVoltage(6))
  }

  private Command moveToHeight(double height) {
    return runEnd(
      () -> setHeight(height),
      () -> {}// elevatorMotor.set(0)
    ).until(isAtTargetHeight);
  }

  public Command moveToHeightUnfoldHighPrecision(double height){
    return run(()->{
        var ff = elevatorFF.getKg();
        elevatorMotor
            .getClosedLoopController()
            .setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
        setpoint.height = height;
      })
      .until(()->MathUtil.isNear(setpoint.height, getCarriageHeight().in(Inches), toleranceHeightUnfolding))
      ;  
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
  
  //DEPRICATED: Left here for debugging
  public Command moveToAngle(double angle) {
    return startRun(
        () -> {
          slewRateAngle.reset(getAngle().in(Degrees));
        },
        () -> {
          setAngle(angle);
        }
      // exit condition on arriving?
    ).until(isAtTargetAngle);
  }

  
  public Command moveToPoseUnchecked(ElevatorPose pose) {
    // return startRun(
    //     () -> {
    //       slewRateAngle.reset(getAngle().in(Degrees));
    //     },
    //     () -> {
    //       setHeight(pose.height);
    //       setAngle(pose.angle);
    //       setScorerSpeed(0);
    //     }
    // // exit condition on arriving?
    // );//.until(isAtTargetPosition); //no good, can't have

    return new InstantCommand(() -> {
      slewRateAngle.reset(getAngle().in(Degrees));
    })
    .andThen(new ParallelCommandGroup(
      moveToAngleTrap(()->pose.angle), //THIS ONE requires elevator subsystem
      new RunCommand(()->{
        setHeight(pose.height);
        setScorerSpeed(0);
      })
    ));
  }

  //Elevator cannot move while arm is at peak speed, too much torque.
  public Command moveToPoseUnchecked(ElevatorPose pose, BooleanSupplier canMoveElevator) {
    return new InstantCommand(() -> {
      slewRateAngle.reset(getAngle().in(Degrees));
    })
    .andThen(new ParallelCommandGroup(
      moveToAngleTrap(()->pose.angle), //THIS ONE requires elevator subsystem
      new RunCommand(()->{
        if(canMoveElevator.getAsBoolean()){
          setHeight(pose.height);
        }
        setScorerSpeed(0);
      })
    ));
  }

  public Command moveToPoseSafe(ElevatorPose pose) {
    return new SequentialCommandGroup(
      moveToPoseUnchecked(kStowedUp).until(isAtSafePosition),
      // moveToPoseUnchecked(pose)
      new ParallelDeadlineGroup(
        moveToPoseUnchecked(pose, ()->getAngularVelocity().in(DegreesPerSecond)<45)
        // realignCoralScorer()
      )
    );
  }
  public Command scoreAtPoseSafe(ElevatorPose pose) {
    // return moveToPoseSafe(pose)
    // .andThen(moveToPoseWithScorer(pose));

    return new SequentialCommandGroup(
      moveToPoseUnchecked(kStowedUp).until(isAtSafePosition),
      // moveToPoseUnchecked(pose)
      new ParallelDeadlineGroup(
        moveToPoseWithScorer(pose)
        // realignCoralScorer()
      )
    );
  }


  public Command goToIntake(Trigger isDown){
    return new SequentialCommandGroup(
      moveToPoseSafe(kPrepareToFloorPickup).until(isAtTargetPosition),
      Commands.idle(this).until(isDown),
      moveToPoseUnchecked(kFloorPickup)
    );
  }


  //Move to prepare to floor pickup, wait for intake to lower(also cuz if we rotate straight to intake we ram into support)
  public Command moveToIntake(Trigger lowerIntake){
    return new SequentialCommandGroup(
      moveToPoseSafe(kPrepareToFloorPickup).until(()->isAtPosition((kPrepareToFloorPickup))),
      Commands.idle(this).until(lowerIntake),
      new ParallelCommandGroup(
        moveToHeight(kFloorPickup.height),
        new RunCommand(()->coralOutMotor.setVoltage(6))
        .until(isCoralInScorer)
        // .andThen(new WaitCommand(0.1))
        .finallyDo((e)->{
          // coralOutMotor.setVoltage(-2);
          coralOutMotor.getEncoder().setPosition(0);
        })
      )
    );
    //In theory, move back up at very end. doesnt work, causes things to break, whatever
    // .finallyDo((e)->moveToHeight(kPrepareToFloorPickup.height));
  }

  public Command moveToStationPickup(){
    return new SequentialCommandGroup(
      moveToPoseUnchecked(kStowedUp).until(isAtSafePosition),
      moveToPoseWithScorer(kStationPickup)
    ).until(isCoralInScorer)
    .finallyDo((e)->{
      // coralOutMotor.setVoltage(-2);
      coralOutMotor.getEncoder().setPosition(0);
    });
  }

  public Command pidScorerBack(){
    return new InstantCommand(()->{})//coralOutMotor.getEncoder().setPosition(0))
    .andThen(new RunCommand(()->coralOutMotor.getClosedLoopController().setReference(-2, ControlType.kPosition, ClosedLoopSlot.kSlot1)));
  }

  public Command realignCoralScorer(){
    return Commands.sequence(
      new RunCommand(()->setScorerSpeed(-1200)).onlyWhile(isCoralInScorer),
      new InstantCommand(()->coralOutMotor.getEncoder().setPosition(0)),
      new RunCommand(()->coralOutMotor.getClosedLoopController().setReference(2, ControlType.kPosition, ClosedLoopSlot.kSlot1))
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
    // return startRun(
    //     () -> {
    //       slewRateAngle.reset(getAngle().in(Degrees));
    //     },
    //     () -> {
    //       setHeight(pose.height);
    //       setAngle(pose.angle);
    //       setScorerSpeed(pose.speed);
    //     }
    // );
    // exit condition on arriving?

    return new InstantCommand(() -> {
      slewRateAngle.reset(getAngle().in(Degrees));
    })
    .andThen(new ParallelCommandGroup(
      moveToAngleTrap(()->pose.angle), //THIS ONE requires elevator subsystem
      new RunCommand(()->{
        setHeight(pose.height);
        setScorerSpeed(pose.speed);
      })
    ));
  }

  public Command holdPosition(){
    return run(
      () -> {
        setHeight(getCarriageHeight().in(Inches));
        setAngle(getAngle().in(Degrees)); //intentionally not trapped, holding angle basepid
        setScorerSpeed(coralOutMotor.getEncoder().getVelocity());
      }
    );
  }

  public Trigger isAtTargetAngle = 
    new Trigger(()->{
      return MathUtil.isNear(setpoint.angle, getAngle().in(Degrees), toleranceAngle);
    })
  ;

  public Trigger isAtTargetHeight = 
    new Trigger(()->{
      return MathUtil.isNear(setpoint.height, getCarriageHeight().in(Inches), toleranceHeight);
    })
  ;

  public Trigger isAtTargetPosition = isAtTargetAngle.and(isAtTargetHeight).debounce(0.02*3);

  public Trigger isClear = new Trigger(()->getAngle().in(Degree) > 85.0
    && setpoint.angle > 85.0
  )
  .or(()->getCarriageHeight().in(Inches) > kPrepareToFloorPickup.height - 1
    && setpoint.height > kPrepareToFloorPickup.height - 1
  )
  .or(()->getCarriageHeight().in(Inches)>12.7 && getAngle().in(Degrees) > 50);


  public boolean isAtPosition(ElevatorPose pose){
    return MathUtil.isNear(pose.angle, getAngle().in(Degrees), toleranceAngle) && 
    MathUtil.isNear(pose.height, getCarriageHeight().in(Inches), toleranceHeight);

  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("elevator/height", getCarriageHeight().in(Inches));
    // SmartDashboard.putNumber("elevator/setpoint", setpoint.height);
    // SmartDashboard.putNumber("elevator/appliedvoltage", elevatorMotor.getAppliedOutput()*rotationMotor.getBusVoltage());
    // SmartDashboard.putNumber("elevator/current", elevatorMotor.getOutputCurrent());
    
    
    // // SmartDashboard.putNumber("elevator/out-motor/position", coralOutMotor.getEncoder().getPosition());
    // // SmartDashboard.putNumber("elevator/out-motor/voltage", coralOutMotor.getEncoder().getVelocity());

    // SmartDashboard.putNumber("elevator/angle/Current", rotationMotor.getOutputCurrent());
    SmartDashboard.putNumber("elevator/rotation/angle", getAngle().in(Degree));
    SmartDashboard.putNumber("elevator/rotation/absoluteAngel", getAngleAbsolute().in(Degree));
    // SmartDashboard.putNumber("elevator/rotation/velocity", rotationMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("elevator/angleVoltage",rotationMotor.getAppliedOutput()*rotationMotor.getBusVoltage());
    // SmartDashboard.putNumber("elevator/inputVoltage",rotationMotor.getBusVoltage());

    // SmartDashboard.putNumber("elevator/rotation/plot/setpointNrml", setpoint.angle);
    // SmartDashboard.putNumber("elevator/rotation/plot/setpointTrap", armSetpoint.position);
    
    // SmartDashboard.putNumber("elevator/lasercanDistance", laserCan.getDistanceOptional().orElse(Inches.of(99999)).in(Inches));
    // SmartDashboard.putBoolean("elevator/lasercanHAsGamepiece", isCoralInScorer.getAsBoolean());
    // SmartDashboard.putBoolean("elevator/isClear", isClear.getAsBoolean());

    // SmartDashboard.putData("elevator/lasercan", laserCan);
    // SmartDashboard.putData("elevator/lasercan", laserCan.sen);
    
    mechanism.update(
      //TODO: Make this intake a CoralInputs object and read from that
      getCarriageHeight().in(Inches),
      getAngle().in(Degree),
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
