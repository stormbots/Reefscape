// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.google.errorprone.annotations.CheckReturnValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
  public boolean isHomed = false;
  public double power;
  public int current;
  private final double toleranceHeight = 0.5;
  private final double toleranceAngle = 5;
  ElevatorPose setpoint = new ElevatorPose(0, 0, 0);

  SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMax rotationMotor = new SparkMax(12, MotorType.kBrushless);
  SparkMax coralOutMotor = new SparkMax(13, MotorType.kBrushless);
  SparkMax elevatorMotorFollower = new SparkMax(11, MotorType.kBrushless);

  SparkMaxSim simElevatorMotor = new SparkMaxSim(elevatorMotor, DCMotor.getNeoVortex(1));
  SparkMaxSim simRotationMotor = new SparkMaxSim(rotationMotor, DCMotor.getNeoVortex(1));
  SparkMaxSim simCoralOutMotor = new SparkMaxSim(coralOutMotor, DCMotor.getNeoVortex(1));

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
  public final ElevatorPose kFloorPickup =    new ElevatorPose(0, 15, -10);
  public final ElevatorPose kStowed =         new ElevatorPose(0, 90, 0);
  public final ElevatorPose kClimbing =       new ElevatorPose(0, 90, 0);
  public final ElevatorPose kL1 =             new ElevatorPose(24, 90, 10);
  public final ElevatorPose kL2 =             new ElevatorPose(30, 135, 10);
  public final ElevatorPose kL3 =             new ElevatorPose(36, 135, 10);
  public final ElevatorPose kL4 =             new ElevatorPose(40, 135, 10);

  SparkBaseConfig elevatorHighPowerConfig = new SparkMaxConfig().smartCurrentLimit(40);

  ArmFeedforward rotatorFF = new ArmFeedforward(0, 0, 0, 0);
  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 0.79, 0);
  public Elevator() {

    // define motor, and set soft limits set up motors so they dont do wierd stuff

    SparkBaseConfig elevatorConfig = new SparkMaxConfig()
      .smartCurrentLimit(36)
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      ;
    elevatorConfig.softLimit
    .forwardSoftLimit(40)//???? This is not enables and wont be tuned until we have compbot
    .reverseSoftLimit(0)
    .reverseSoftLimitEnabled(true);
    ;
    var elevatorConversionfactor = (51 - 12)/22.11;
    elevatorConfig.encoder
        .positionConversionFactor(elevatorConversionfactor)
        .velocityConversionFactor(elevatorConversionfactor / 60.0)
        ;
    elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1);





    SparkBaseConfig coralOutConfig = new SparkMaxConfig()
      .smartCurrentLimit(16)
      .idleMode(IdleMode.kCoast)
      .inverted(false); //Positive is in, Negative is out

    var coralOutConversionFactor = 3.371;
    coralOutConfig.encoder
      .velocityConversionFactor(coralOutConversionFactor / 60.0)
      .positionConversionFactor(coralOutConversionFactor)
      ;

    coralOutConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0);





    SparkBaseConfig rotationConfig = new SparkMaxConfig()
      .smartCurrentLimit(8)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      ;

    rotationConfig.softLimit
      .reverseSoftLimit(-180)
      .forwardSoftLimit(180)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false)
      ;

    double rotateCoversionFactor = 87.053/2.5333;
    rotationConfig.encoder
        .velocityConversionFactor(rotateCoversionFactor / 60.0)
        .positionConversionFactor(rotateCoversionFactor)
        ;        

    rotationConfig.absoluteEncoder
      .velocityConversionFactor(360 / 60.0)
      .positionConversionFactor(360)
      .inverted(false);

    rotationConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(0.8/60.0)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 360)
      ;

    var elevatorFollowerConfig = new SparkFlexConfig()
      .apply(elevatorConfig)
      .follow(elevatorMotor,true);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralOutMotor.configure(coralOutConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rotationMotor.getEncoder().setPosition(rotationMotor.getAbsoluteEncoder().getPosition());
    // new Trigger(DriverStation::isEnabled).and(() -> isHomed == false).onTrue(homeElevator());

    new Trigger(DriverStation::isDisabled).onTrue(runOnce(()->rotationMotor.stopMotor()));

    simInit();
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

  private void setHeight(double height) {
    var ff = elevatorFF.getKg();
    elevatorMotor
        .getClosedLoopController()
        .setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    setpoint.height = height;
  }

  private void setAngle(double angle) {
    var ff = rotatorFF.calculate(Math.toRadians(angle), 0);
    ff=0;//FIXME
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

  public Command moveToPose(ElevatorPose pose) {
    return run(
        () -> {
          setHeight(pose.height);
          setAngle(pose.angle);
          setScorerSpeed(0);
        })
    // exit condition on arriving?
    ;
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
    return run(
        () -> {
          setAngle(angle);
        })
    // exit condition on arriving?
    ;
  }

  // public Command Score(double voltage) {
  //   return runEnd(
  //     ()-> {
  //       coralOutMotor.setVoltage(voltage);
  //     },
  //     () -> {
  //       coralOutMotor.setVoltage(0);
  //     }
  //   );
  // }


  private Command moveToPoseWithScorer(ElevatorPose pose) {
    return run(
        () -> {
          setHeight(pose.height);
          setAngle(pose.angle);
          setScorerSpeed(pose.speed);
        })
    // exit condition on arriving?
    ;
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
    SmartDashboard.putNumber("elevaor/angle/degres", Units.radiansToDegrees(simArm.getAngleRads()));

    SmartDashboard.putNumber("elevator/angle/Current", rotationMotor.getOutputCurrent());
    SmartDashboard.putNumber("elevator/angleVoltage",rotationMotor.getAppliedOutput()*rotationMotor.getBusVoltage());
    SmartDashboard.putNumber("elevator/inputVoltage",rotationMotor.getBusVoltage());
    mechanism.mechanismUpdate();
  }

  /////////////////////////////
  /// Do the mechanism logging
  /////////////////////////////
  // Create the basic mechanism construction
  class ElevatorMechanism {
    double angledelta = 15;
    double barlength = 24;

    public Mechanism2d mech = new Mechanism2d(36, 72);
    MechanismRoot2d root = mech.getRoot("ElevatorRoot", 18, 0);
    MechanismLigament2d elevator = root.append(new MechanismLigament2d("Elevator", 7, 90));
    //This only exists to re-orient the rotator to a horizontal reference so later setAngles make sense
    MechanismLigament2d rotatorMount = elevator.append(new MechanismLigament2d("RotatorBase", 0, -90));
    MechanismLigament2d rotator = rotatorMount.append(new MechanismLigament2d("Rotator", 1, 0));
    MechanismLigament2d coral = rotator.append(new MechanismLigament2d("ElevatorCoral", 1, 0));
    MechanismLigament2d translator = rotator.append(new MechanismLigament2d("Translator", 0, 0));
    //These are just fixed rigid bars for visual reference
    MechanismLigament2d translatorBarA = rotator.append(new MechanismLigament2d("ATranslatorBarA", 12, 0));
    MechanismLigament2d translatorBarB = rotator.append(new MechanismLigament2d("ATranslatorBarB", -6, 0));
    //Visualize the rotator's relative encoder
    MechanismLigament2d rotatorRelative = rotatorMount.append(new MechanismLigament2d("RotatorRelative", 13, 0));

    private ElevatorMechanism() {
      var barweight = 10;

      elevator.setColor(new Color8Bit(Color.kDarkGray));
      elevator.setLineWeight(barweight * 2);

      rotator.setColor(new Color8Bit(Color.kDarkGreen));
      rotator.setLineWeight(barweight);

      translatorBarA.setColor(new Color8Bit(Color.kDarkGray));
      translatorBarA.setLineWeight(barweight/2);
      translatorBarB.setColor(new Color8Bit(Color.kDarkGray));
      translatorBarB.setLineWeight(barweight/2);

      translator.setColor(new Color8Bit(Color.kDarkRed));
      translator.setLineWeight(barweight * 2);

      coral.setColor(new Color8Bit(Color.kWhite));
      coral.setLineWeight(barweight * 4);
      coral.setLength(0);

      rotatorRelative.setColor(new Color8Bit(Color.kRed));
      rotatorRelative.setLineWeight(barweight/2-1);

      SmartDashboard.putData("mechanism/elevator", mech);
    }

    private void mechanismUpdate() {
      var height = elevatorMotor.getEncoder().getPosition();
      var angle = rotationMotor.getAbsoluteEncoder().getPosition();
      var translatespeed = coralOutMotor.getEncoder().getVelocity();
      var isCoralLoaded = haveCoral.getAsBoolean();

      elevator.setLength(height);
      rotator.setAngle(new Rotation2d(Math.toRadians(angle)));
      translator.setLength(translatespeed / 5760 * 12);
      // coral.setLength(isCoralLoaded ? 12 : 0);
    }
  }
  ElevatorMechanism mechanism = new ElevatorMechanism();


  ElevatorSim simElevator = new ElevatorSim(
    DCMotor.getNeoVortex(1),
    18, 
    1, 
    Inches.of(1).in(Meter), 
    Inches.of(0).in(Meter), 
    Inches.of(48).in(Meter), 
    true, 
    0
  );
  
  double armlength = 0.3;//m
  SingleJointedArmSim simArm = new SingleJointedArmSim(
    DCMotor.getNeoVortex(1), 
    18*4, 
    2*armlength*armlength/3.0, 
    armlength, 
    Degrees.of(-90).in(Radians), 
    Degrees.of(180).in(Radians), 
    true, 
    Degrees.of(90).in(Radians)
    );

  public void simInit(){
    simArm.setState(Degrees.of(75).in(Radians), 0);
  }

  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      var vbus = 12;
      var dt = 0.02;

      simElevator.setInputVoltage(simElevatorMotor.getAppliedOutput()*vbus);
      simElevator.update(dt);

      simArm.setInputVoltage(simRotationMotor.getAppliedOutput()*vbus);
      simArm.update(dt);
      SmartDashboard.putNumber("arm/plant/angle", Math.toDegrees(simArm.getAngleRads()));

      simElevatorMotor.iterate(
        MetersPerSecond.of(simElevator.getVelocityMetersPerSecond()).in(InchesPerSecond),
        vbus,
        dt
      );

      simRotationMotor.iterate(
        RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond),
        vbus,
        dt
      );
      // simRotationMotor.iterate(velocity, vbus, dt);
      // simCoralOutMotor.iterate(velocity, vbus, dt);

  }
}
