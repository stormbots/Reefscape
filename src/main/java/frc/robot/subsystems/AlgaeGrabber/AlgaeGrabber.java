// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class AlgaeGrabber extends SubsystemBase {
  // public final AlgaeGrabberIO io;
  // public final AlgaeGrabberIOInputsAutoLogged inputs = new AlgaeGrabberIOInputsAutoLogged();

  SparkMax shooterMotor = new SparkMax(1, MotorType.kBrushless);
  SparkMax armMotor = new SparkMax(5, MotorType.kBrushless);
  SparkMax rollerMotor = new SparkMax(3, MotorType.kBrushless);

  private SlewRateLimiter armAngleSlew = new SlewRateLimiter(90);

  public static final double ROLLERHOLDPOWER = 0.2;
  public static final double ROLLERINTAKERPM = 0.5;
  public static final double ROLLEREJECTPOWER = -0.5;

  private static final double ANGLETOLERANCE = 4;
  private double RPMTOLERANCE = 0;

  private double angleSetpoint = -90;
  private double shooterRPMSetpoint = 0;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {
    // this.io = io;

    var armConf = new SparkMaxConfig();
    armConf.inverted(false);

    armConf
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0)
        .inverted(false);
    armConf.closedLoop.p(0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armMotor.configure(armConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // add roller conf

    var rollerConf = new SparkMaxConfig();
    rollerConf.inverted(false);

    rollerConf
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0)
        .inverted(false);

    // add shooter conf?

    var shooterConf = new SparkMaxConfig();
    shooterConf.inverted(false);

    shooterConf
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0)
        .inverted(false);

    setDefaultCommand(defaultCommand());
  }

  //////////////////////////////////
  /// Define some helpful motor function
  /// ///////////////////////////////

  private void setShooterRPM(double rpm) {
    this.shooterRPMSetpoint = rpm;
    shooterMotor
        .getClosedLoopController()
        .setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
  }

  private void setIntakeRPM(double rpm) {
    this.shooterRPMSetpoint = rpm;
    rollerMotor
        .getClosedLoopController()
        .setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
  }

  private void setArmAngle(double angle) {
    this.angleSetpoint = angle;

    // one: profiledPid controller -> runs on roborio
    // two: SparkMotionMagic
    // three: Temp solution, slew rate on target value
    angle = armAngleSlew.calculate(angle);

    armMotor
        .getClosedLoopController()
        .setReference(
            angle,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            0, // TODO: Add me
            ArbFFUnits.kVoltage);
  }

  private double getAngle() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  private double getShooterRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }

  //////////////////////////////////
  /// Define some commands
  /// ///////////////////////////////

  public Command defaultCommand() {
    return run(
        () -> {
          setArmAngle(0);
          if (haveAlgae) {
            setIntakeRPM(ROLLERHOLDPOWER);
          } else {
            setIntakeRPM(0);
          }
        });
  }

  public final double HAVEGRABBEDALGAEAMPS = 5;
  private boolean haveAlgae = false;

  public Command intakeAlgaeFromFloor() {
    // set arm to grab angle
    // set rollers to grab
    // when aquired, end
    return run(() -> {
          setArmAngle(0);
          setIntakeRPM(ROLLERINTAKERPM);
        })
        .until(() -> rollerMotor.getOutputCurrent() > HAVEGRABBEDALGAEAMPS)
        .finallyDo(
            (interrupted) -> {
              if (interrupted == false) {
                haveAlgae = true;
              }
            });
    // .ifexitsuccessfully(haveAlgae = true)
  }

  public Command prepareToShoot() {
    //  if HAVEALGAE ->
    // move arm from bottom to shooter theta
    return run(
        () -> {
          setArmAngle(-90 + 30);
          // if we have shooters, spin up
          setShooterRPM(ROLLEREJECTPOWER); // do we spin up shooters?
        });
  }

  public Command scoreProcessor() {
    return scoreAlgae(() -> -90, () -> 100);
  }

  private Command shootAlgaeUnchecked(double targetRPM) {
    return run(() -> {
          setShooterRPM(targetRPM);
          setIntakeRPM(targetRPM);
        })
        .finallyDo(
            (interrupted) -> {
              if (interrupted == true) {
                haveAlgae = false;
              }
            });
  }

  public Command scoreInNetEzMode() {
    return scoreAlgae(() -> 140, () -> 1);
  }

  public Command scoreAlgae(DoubleSupplier angle, DoubleSupplier rpm) {
    // Move arm to the proper angle
    // until(is at position)
    // and then actually shoot
    return new SequentialCommandGroup(
        prepareToShoot().until(isAtTargetAngle.and(isAtTargetRPM)),
        shootAlgaeUnchecked(rpm.getAsDouble()).withTimeout(0.5));
  }

  public Command scoreNetPose(Pose2d pose) {
    // calc distance to net
    // LUT(distance) -> angle and/or rpm

    // arm at scoring angle
    // spin up shooter to required rpm
    // shoot out
    return runOnce(() -> {});
  }

  public Trigger isAtTargetAngle =
      new Trigger(
          () -> {
            return bounded(
                getAngle(), angleSetpoint - ANGLETOLERANCE, angleSetpoint + ANGLETOLERANCE);
          });

  public Trigger isAtTargetRPM =
      new Trigger(
          () -> {
            return bounded(
                getShooterRPM(),
                shooterRPMSetpoint - RPMTOLERANCE,
                shooterRPMSetpoint + RPMTOLERANCE);
          });

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // io.updateInputs(inputs);
    // Logger.processInputs("AlgaeGrabber", inputs);
  }

  private boolean bounded(double input, double min, double max) {
    if (input < min) return false;
    if (input > max) return false;
    return true;
  }
}
