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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrabber extends SubsystemBase {
  public final AlgaeGrabberIO io;
  // public final AlgaeGrabberIOInputsAutoLogged inputs = new AlgaeGrabberIOInputsAutoLogged();

  SparkMax shooterMotor = new SparkMax(1, MotorType.kBrushless);
  SparkMax armMotor = new SparkMax(5, MotorType.kBrushless);
  SparkMax rollerMotor = new SparkMax(3, MotorType.kBrushless);

  private SlewRateLimiter armAngleSlew = new SlewRateLimiter(90);

  public static final double ROLLERHOLDPOWER = 0.2;
  public static final double ROLLERINTAKEPOWER = 0.5;
  public static final double ROLLEREJECTPOWER = -0.5;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber(AlgaeGrabberIO io) {
    this.io = io;


    var armConf = new SparkMaxConfig();
    armConf.inverted(false);

    armConf
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0)
        .inverted(false);
    armConf.closedLoop.p(0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armMotor.configure(armConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(defaultCommand());
  }

  private void setArmAngle(double angle) {
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

  public Command defaultCommand() {
    return run(
        () -> {
          setArmAngle(0);
          if (haveAlgae) {
            rollerMotor.set(ROLLERHOLDPOWER);
          } else {
            rollerMotor.set(0);
          }
        });
  }

  public final double HAVEGRABBEDALGAEAMPS = 5;
  private boolean haveAlgae = false;

  public Command grabAlgae() {
    // set arm to grab angle
    // set rollers to grab
    // when aquired, end
    return run(() -> {
      setArmAngle(50);
      rollerMotor.set(1);
    })
    .until(() -> rollerMotor.getOutputCurrent() > HAVEGRABBEDALGAEAMPS)
    .finallyDo( (interrupted) -> {
        if (interrupted ==false ) {
          haveAlgae = true;
        }
      })
    // .ifexitsuccessfully(haveAlgae = true)
    ;
  }

  public void pivotToShooter(){
    //  if HAVEALGAE ->
    // move arm from bottom to shooter theta
    // shoot out
    // return arm to bottom

    
  }
  

  public void scoreProcessor() {
    // arm at neutral angle
    // rollers out
  }

  public void scoreNetManualSimple() {
    // arm at scoring angle
    // spin up shooter to required rpm
    // shoot out
  }

  public void scoreNetManualComplex(double angle, double rpm) {
    // arm at scoring angle
    // spin up shooter to required rpm
    // shoot out
  }

  public void scoreNetPose(Pose2d pose) {
    // calc distance to net
    // LUT(distance) -> angle and/or rpm

    // arm at scoring angle
    // spin up shooter to required rpm
    // shoot out
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // io.updateInputs(inputs);
    // Logger.processInputs("AlgaeGrabber", inputs);
  }
}
