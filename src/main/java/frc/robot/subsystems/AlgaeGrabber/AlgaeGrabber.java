// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrabber extends SubsystemBase {
  // public final AlgaeGrabberIO io;
  // public final AlgaeGrabberIOInputsAutoLogged inputs = new AlgaeGrabberIOInputsAutoLogged();

  SparkFlex shooterMotor = new SparkFlex(16, MotorType.kBrushless);
  SparkFlex armMotor = new SparkFlex(15, MotorType.kBrushless);
  SparkFlex intakeMotor = new SparkFlex(14, MotorType.kBrushless);

  private SlewRateLimiter armAngleSlew = new SlewRateLimiter(90);

  public static final double absconversionfactor=360*1.5/2.0; //account for gearing on the abs encoder

  public static final double ROLLERHOLDPOWER = 0.2;
  public static final double ROLLERINTAKERPM = 2000;
  public static final double ROLLEREJECTPOWER = -0.5;

  private static final double ANGLETOLERANCE = 5;

  private static final double LOWERSOFTLIMIT = -90;
  private static final double UPPERSOFTLIMIT = 10;

  private static final double RPMTOLERANCE = 100;

  private double angleSetpoint = -90;
  private double shooterRPMSetpoint = 0;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {
    // this.io = io;

    var armConf = new SparkMaxConfig()
      .inverted(true)
      .smartCurrentLimit(10)
      .closedLoopRampRate(.2)
      .idleMode(IdleMode.kBrake)
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
    .forwardSoftLimit(UPPERSOFTLIMIT).forwardSoftLimitEnabled(false)
    .reverseSoftLimit(LOWERSOFTLIMIT).reverseSoftLimitEnabled(false)
    ;

    armConf.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(0, absconversionfactor)
    .p(0.2/60.0)
    ;

    armMotor.configure(armConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // add roller conf

    var rollerConf = new SparkMaxConfig();
    rollerConf.inverted(false);

    rollerConf.closedLoop
    // .p(1/500.0)
    .velocityFF(1/5760.0)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    intakeMotor.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // add shooter conf?

    var shooterConf = new SparkMaxConfig();
    shooterConf.inverted(false);

    shooterConf.closedLoop
    // .p(1/500.0)
    .velocityFF(1/5760.0)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shooterMotor.configure(shooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armMotor.getEncoder().setPosition(getAngle());
    setDefaultCommand(defaultCommand());
    //Automatically reset the slew rate if the bot is disabled
    new Trigger(DriverStation::isEnabled)
        .onTrue(new InstantCommand(()->armAngleSlew.reset(getAngle())));
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

  public double getAngle() {
    var angle = armMotor.getAbsoluteEncoder().getPosition();
    if(angle > absconversionfactor/2.0){
      angle = angle - absconversionfactor;
    }
    return angle;
  }

  private double getShooterRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }

  private double getIntakeRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }

  //////////////////////////////////
  /// Define some commands
  /// ///////////////////////////////
  public Command defaultCommand() {
    return run( () -> {
        setShooterRPM(0);
        setArmAngle(-90);
        if (haveAlgae) {
            setIntakeRPM(400);
        } else {
            setIntakeRPM(0);
        }
    }).withName("DefaultCommand");
  }

  public final double HAVEGRABBEDALGAEAMPS = 5;
  private boolean haveAlgae = false;
  Trigger intakeStalled = new Trigger(()->intakeMotor.getOutputCurrent() > HAVEGRABBEDALGAEAMPS)
    .and(()->intakeMotor.getEncoder().getVelocity()<100)    
    .debounce(0.1);

  public Command intakeAlgaeFromFloor() {
    return run(() -> {
      setArmAngle(-36);
      setIntakeRPM(ROLLERINTAKERPM);
    })
    .until(intakeStalled)
    .finallyDo((interrupted) -> {
        if (interrupted == false) {
            haveAlgae = true;
        }
    }).withName("IntakeFromFloor");
  }

  public Command prepareToShoot() {
    return prepareToShoot(()->1000);
  }

  public Command prepareToShoot(DoubleSupplier rpm) {
    return run(() -> {
      setArmAngle(-90 + 30);
      setShooterRPM(rpm.getAsDouble());
    }).withName("PrepareToShoot");
  }

  public Command scoreProcessor() {
    return scoreAlgae(() -> -90, () -> 500)
    .withName("ScoreProcessor");
  }

  public Command shootAlgaeUnchecked(double targetRPM) {
    return run(() -> {
        setShooterRPM(targetRPM);
        setIntakeRPM(targetRPM);
    })
    .finallyDo( (interrupted) -> {
        if (interrupted == false) {
          haveAlgae = false;
        }
    });
  }

  public Command scoreInNetEzMode() {
    return scoreAlgae(() -> -90+45, () -> 500)
    .withName("ScoreInNetEasyMode");
  }

  public Command scoreAlgae(DoubleSupplier angle, DoubleSupplier rpm) {
    // Move arm to the proper angle
    // until(is at position)
    // and then actually shoot
    return new SequentialCommandGroup(
        prepareToShoot(rpm).until(isAtTargetAngle.and(isAtTargetRPM)),
        shootAlgaeUnchecked(rpm.getAsDouble()).withTimeout(0.5)
      ).withName("ScoreAlgae");
  }

  public Command scoreNetPose(Pose2d pose) {
    // calc distance to net
    // LUT(distance) -> angle and/or rpm

    // arm at scoring angle
    // spin up shooter to required rpm
    // shoot out
    return runOnce(() -> {});
  }

  public Trigger isAtTargetAngle = new Trigger(() -> {
    return MathUtil.isNear(angleSetpoint, getAngle(), ANGLETOLERANCE);
  }).debounce(0.060);

  public Trigger isAtTargetRPM = new Trigger(() -> {
    return MathUtil.isNear(shooterRPMSetpoint, getShooterRPM(), RPMTOLERANCE);
  }).debounce(0.060);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // io.updateInputs(inputs);
    // Logger.processInputs("AlgaeGrabber", inputs);
    mechanism.update();//TODO: Make this so it takes an Inputs object

    var currentcommand = Optional
        .ofNullable(this.getCurrentCommand())
        .orElse(Commands.idle())
        .getName();
    SmartDashboard.putString("algae/command", currentcommand);


    SmartDashboard.putNumber("algae/arm angle rel", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("algae/arm angle abs", getAngle());
    // SmartDashboard.putNumber("algae/intake roller pos", intakeMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("algae/shooter roller pos", shooterMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("algae/arm output",armMotor.getOutputCurrent());
    SmartDashboard.putNumber("algae/arm applied output",armMotor.getAppliedOutput());
    SmartDashboard.putNumber("algae/shooter velocity",shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("algae/intake velocity",intakeMotor.getEncoder().getVelocity());
  }

  public AlgaeSim sim = new AlgaeSim(armMotor,intakeMotor,shooterMotor);
  @Override
  public void simulationPeriodic() {
    sim.update();
  }

  AlgaeMech2d mechanism = new AlgaeMech2d(
    ()->Degrees.of(getAngle()),
    ()->RPM.of(getShooterRPM()),
    ()->RPM.of(getIntakeRPM()),
    sim::getAngle
  );

}
