// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO Sync encoders so soft limits work

package frc.robot.subsystems.Elevator;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
  private final double tolerance = 0.1;
  ElevatorPose setpoint = new ElevatorPose(0, 0, 0);

  SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMax rotationMotor = new SparkMax(12, MotorType.kBrushless);
  SparkMax coralOutMotor = new SparkMax(13, MotorType.kBrushless);
  SparkMax elevatorMotorFollower = new SparkMax(11, MotorType.kBrushless);

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

  public final ElevatorPose L1 = new ElevatorPose(24, 0, 10);
  public final ElevatorPose L2 = new ElevatorPose(30, 0, 10);
  public final ElevatorPose L3 = new ElevatorPose(36, 0, 10);
  public final ElevatorPose L4 = new ElevatorPose(40, 0, 10);

  SparkBaseConfig elevatorHighPowerConfig = new SparkMaxConfig().smartCurrentLimit(40);

  public Elevator() {

    // define motor, and set soft limits set up motors so they dont do wierd stuff

    SparkBaseConfig elevatorConfig =
        new SparkMaxConfig().smartCurrentLimit(8).idleMode(IdleMode.kBrake).inverted(false);

    var elevatorConversionfactor = 1.0;
    elevatorConfig
        .encoder
        .positionConversionFactor(elevatorConversionfactor)
        .velocityConversionFactor(elevatorConversionfactor / 60.0)
        ;
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0);

    SparkBaseConfig coralOutConfig =
        new SparkMaxConfig().smartCurrentLimit(8).idleMode(IdleMode.kCoast).inverted(false);

    var coralOutConversionFactor = 1.0;
    coralOutConfig.encoder.velocityConversionFactor(coralOutConversionFactor / 60.0)
    ;

    coralOutConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0);

    SparkBaseConfig rotataionConfig =
        new SparkMaxConfig().smartCurrentLimit(8).idleMode(IdleMode.kBrake).inverted(false);

    var rotateCoversionFactor = 1.0;
    rotataionConfig
        .absoluteEncoder
        .velocityConversionFactor(rotateCoversionFactor / 60.0)
        .positionConversionFactor(rotateCoversionFactor)
        .inverted(false);

    rotataionConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(0);

    var elevatorFollowerConfig = new SparkFlexConfig()
    .apply(elevatorConfig)
    .follow(elevatorMotor,true);

    elevatorMotor.configure(
      elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralOutMotor.configure(
      coralOutConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rotationMotor.configure(
      rotataionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorMotorFollower.configure(
      elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    new Trigger(DriverStation::isEnabled).and(() -> isHomed == false).onTrue(homeElevator());
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
    elevatorMotor
        .getClosedLoopController()
        .setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
    setpoint.height = height;
  }

  private void setAngle(double angle) {
    rotationMotor
        .getClosedLoopController()
        .setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
    setpoint.angle = angle;
  }

  private void setScorerSpeed(double speed) {
    rotationMotor
        .getClosedLoopController()
        .setReference(speed, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
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
        .andThen(scoreAtPose(pose));
  }

  public Trigger atTargetPosition =
      new Trigger(
          () -> {
            if (bounded(
                    elevatorMotor.getEncoder().getPosition(),
                    setpoint.height - tolerance,
                    setpoint.height + tolerance)
                && bounded(
                    rotationMotor.getAbsoluteEncoder().getPosition(),
                    setpoint.angle - tolerance,
                    setpoint.angle + tolerance)) {
              return true;
            }
            return false;
          });

  public boolean bounded(double value, double min, double max) {
    if (value >= min && value <= max) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Elevator Height", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Scorer Angle", coralOutMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Coral out Speed", rotationMotor.getAbsoluteEncoder().getPosition());

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
    MechanismLigament2d rotator = rotatorMount.append(new MechanismLigament2d("Rotator", 0, 0));
    MechanismLigament2d coral = rotator.append(new MechanismLigament2d("ElevatorCoral", 0, 0));
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
      coral.setLength(isCoralLoaded ? 12 : 0);
    }
  }

  ElevatorMechanism mechanism = new ElevatorMechanism();
}
