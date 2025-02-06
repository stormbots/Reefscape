// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeGrabber extends SubsystemBase {
  // public final AlgaeGrabberIO io;
  // public final AlgaeGrabberIOInputsAutoLogged inputs = new AlgaeGrabberIOInputsAutoLogged();

  SparkMax shooterMotor = new SparkMax(16, MotorType.kBrushless);
  SparkMax armMotor = new SparkMax(15, MotorType.kBrushless);
  SparkMax intakeMotor = new SparkMax(14, MotorType.kBrushless);

  private SlewRateLimiter armAngleSlew = new SlewRateLimiter(90);

  public static final double absconversionfactor=360*1.5/2.0; //account for gearing on the abs encoder

  public static final double ROLLERHOLDPOWER = 0.2;
  public static final double ROLLERINTAKERPM = 0.5;
  public static final double ROLLEREJECTPOWER = -0.5;

  private static final double ANGLETOLERANCE = 4;

  private static final double LOWERSOFTLIMIT = -90;
  private static final double UPPERSOFTLIMIT = 10;

  private double RPMTOLERANCE = 0;

  private double angleSetpoint = -90;
  private double shooterRPMSetpoint = 0;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {
    // this.io = io;

    var armConf = new SparkMaxConfig()
      .inverted(true)
      .smartCurrentLimit(5);
      ;

    armConf.absoluteEncoder
        .positionConversionFactor(absconversionfactor)//This is on a 1:1.5 gear step
        .velocityConversionFactor(absconversionfactor / 60.0)
        .inverted(false);
    var conversionfactor=(90/30.362);
    armConf.encoder
    .positionConversionFactor(conversionfactor)
    .velocityConversionFactor(conversionfactor/60.0);
    ;
    armConf.softLimit
    .forwardSoftLimit(UPPERSOFTLIMIT).forwardSoftLimitEnabled(true)
    .reverseSoftLimit(LOWERSOFTLIMIT).reverseSoftLimitEnabled(true)
    ;

    armConf.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(0, absconversionfactor)
    .p(0)
    ;

    armMotor.configure(armConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // add roller conf

    var rollerConf = new SparkMaxConfig();
    rollerConf.inverted(false);

    rollerConf
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0)
        .inverted(false);

    rollerConf.closedLoop.p(0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    intakeMotor.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // add shooter conf?

    var shooterConf = new SparkMaxConfig();
    shooterConf.inverted(false);

    shooterConf
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0)
        .inverted(false);

    shooterConf.closedLoop.p(0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    shooterMotor.configure(shooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armMotor.getEncoder().setPosition(getAngle());
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
    intakeMotor
        .getClosedLoopController()
        .setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
  }

  private void setArmAngle(double angle) {
    angle = MathUtil.clamp(angle, LOWERSOFTLIMIT, UPPERSOFTLIMIT);

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
    var angle = armMotor.getAbsoluteEncoder().getPosition();
    if(angle > absconversionfactor/2.0){
      angle = angle - absconversionfactor;
    }
    return angle;
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
        .until(() -> intakeMotor.getOutputCurrent() > HAVEGRABBEDALGAEAMPS)
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
    mechanism.mechanismUpdate();

    SmartDashboard.putNumber("algae/arm angle rel", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("algae/arm angle abs", getAngle());
    SmartDashboard.putNumber("algae/intake roller pos", intakeMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("algae/shooter roller pos", shooterMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("algae velocity",shooterMotor.getEncoder().getVelocity());
  }

  private boolean bounded(double input, double min, double max) {
    if (input < min) return false;
    if (input > max) return false;
    return true;
  }

  /////////////////////////////
  /// Do the mechanism logging
  /////////////////////////////
  // Create the basic mechanism construction
  class AlgaeMechanism {
    double angledelta = 15;
    double barlength = 24;

    public Mechanism2d mech = new Mechanism2d(36, 72);
    MechanismRoot2d root = mech.getRoot("AlgaeGrabber", 6, 36);
    MechanismLigament2d intakebarRelative =  root.append(new MechanismLigament2d("AlgaeIntakeBarRelative", 24, -90));
    MechanismLigament2d intakebar = root.append(new MechanismLigament2d("AlgaeIntakeBar", 24, -90));
    MechanismLigament2d shooterbar = root.append(new MechanismLigament2d("AlgaeShooterBar", 24, -90 + 10));
    MechanismLigament2d intake = intakebar.append(new MechanismLigament2d("AlgaeIntake", 0, 90));
    MechanismLigament2d shooter = shooterbar.append(new MechanismLigament2d("AlgaeShooter", 0, 90));

    private AlgaeMechanism() {
      var barweight = 10;
      intakebarRelative.setColor(new Color8Bit(Color.kRed));
      intakebarRelative.setLineWeight(barweight - 1);

      intakebar.setColor(new Color8Bit(Color.kGray));
      shooterbar.setColor(new Color8Bit(Color.kGray));
      intakebar.setLineWeight(barweight);
      shooterbar.setLineWeight(barweight);

      intake.setColor(new Color8Bit(Color.kDarkGreen));
      shooter.setColor(new Color8Bit(Color.kDarkRed));
      intake.setLineWeight(barweight);
      shooter.setLineWeight(barweight);
      SmartDashboard.putData("mechanism/algaegrabber", mech);
    }

    private void mechanismUpdate() {
      var angle = getAngle();
      intakebar.setAngle(new Rotation2d(Math.toRadians(angle)));
      shooterbar.setAngle(new Rotation2d(Math.toRadians(angle + angledelta)));

      intake.setLength(intakeMotor.getEncoder().getVelocity() / 5760.0 * barlength / 4);
      shooter.setLength(shooterMotor.getEncoder().getVelocity() / 5760.0 * barlength / 4);

      // This is mostly to validate absolute vs relative, since they should be identical
      intakebarRelative.setAngle(
          new Rotation2d(Math.toRadians(armMotor.getEncoder().getPosition())));
    }
  }

  AlgaeMechanism mechanism = new AlgaeMechanism();
}
