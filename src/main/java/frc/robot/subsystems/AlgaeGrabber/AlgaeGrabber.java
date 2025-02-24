// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stormbots.LaserCanWrapper;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeGrabber extends SubsystemBase {
  // public final AlgaeGrabberIO io;
  // public final AlgaeGrabberIOInputsAutoLogged inputs = new AlgaeGrabberIOInputsAutoLogged();

  SparkFlex shooterMotor = new SparkFlex(16, MotorType.kBrushless);
  SparkFlex armMotor = new SparkFlex(15, MotorType.kBrushless);
  SparkFlex intakeMotor = new SparkFlex(17, MotorType.kBrushless);

  private SlewRateLimiter armAngleSlew = new SlewRateLimiter(90);
  LaserCanWrapper laserCan = new LaserCanWrapper(23)//TODO: Needs correct id
    .configureShortRange()
    .setThreshhold(Inches.of(6));
  public static final double absconversionfactor=360/2.0; //account for gearing on the abs encoder

  public static final double ROLLERHOLDRPM = 200;
  public static final double ROLLERINTAKERPM = 3500;
  public static final double SHOOTERINTAKERPM = -1200;
  public static final double ROLLEREJECTPOWER = -0.5;

  private static final double ANGLETOLERANCE = 5;

  private static final double LOWERSOFTLIMIT = -90;
  private static final double UPPERSOFTLIMIT = 10;

  private static final double RPMTOLERANCE = 200;

  private double angleSetpoint = -90;
  private double shooterRPMSetpoint = 0;

  //TODO: Find real angles
  private ArmFeedforward armFF = new ArmFeedforward(0.3, 0.25, 0);


  private final TrapezoidProfile armTrapProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(30, 30)
  );

  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();


  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {
    // this.io = io;

    var armConf = new SparkFlexConfig()
      .inverted(false)
      .smartCurrentLimit(30)
      .closedLoopRampRate(.2)
      .idleMode(IdleMode.kBrake)
      ;

    armConf.absoluteEncoder
        .positionConversionFactor(absconversionfactor)//This is on a 2:1 gear step (Brian-2:1 on sprocket)
        .velocityConversionFactor(absconversionfactor / 60.0)
        .inverted(false);
    var conversionfactor=360.0/(45*2.0);//45:1 planetary reduction, 2:1 sprocket
    armConf.encoder
    .positionConversionFactor(conversionfactor)
    .velocityConversionFactor(conversionfactor/60.0);
    ;
    armConf.softLimit
    .forwardSoftLimit(UPPERSOFTLIMIT).forwardSoftLimitEnabled(false)
    .reverseSoftLimit(LOWERSOFTLIMIT).reverseSoftLimitEnabled(false)
    ;

    armConf.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.2*4/60.0)
    ;

    armMotor.configure(armConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // add roller conf

    var rollerConf = new SparkFlexConfig();
    rollerConf
      .inverted(false)
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      ;

    rollerConf.encoder
      //IDEK, velocity filtering but idk whether it reads hall or quaderature primarily and how to decide which one to read
      .uvwMeasurementPeriod(8)
      .uvwAverageDepth(2)
      .quadratureMeasurementPeriod(2) //quaderature is apparently much better tho
      .quadratureAverageDepth(2);
    
    rollerConf.closedLoop
    // .p(1/500.0)
    .velocityFF(1/5760.0)
    .p(0.1*4*2*2*2/9, ClosedLoopSlot.kSlot1)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);


    rollerConf.closedLoopRampRate(0.05);
    rollerConf.openLoopRampRate(0.05);
    
    intakeMotor.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // add shooter conf?

    var shooterConf = new SparkFlexConfig();
    shooterConf
    .inverted(false)
    .smartCurrentLimit(60)
    .idleMode(IdleMode.kCoast)
    ;

    shooterConf.encoder
      .uvwMeasurementPeriod(8)
      .uvwAverageDepth(2)
      .quadratureMeasurementPeriod(2)
      .quadratureAverageDepth(2);

    shooterConf.closedLoop
    // .p(1/500.0)
    .velocityFF(1/5760.0*0.95)
    .p(0.1*4*2*2*2/3, ClosedLoopSlot.kSlot1)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shooterMotor.configure(shooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterConf.closedLoopRampRate(0.05);
    shooterConf.openLoopRampRate(0.05);
    //MUST BE BETWEEN [-135,45] WHEN INITIALIZED, syncs encoders'
    Timer.delay(0.1);
    armMotor.getEncoder().setPosition(getAbsoluteAngleDegrees());

    setDefaultCommand(defaultCommand());
    SmartDashboard.putNumber("algae/ShootingAngle", -10);
    SmartDashboard.putNumber("algae/ShooterVoltage", 0.8);
    //Automatically reset the slew rate if the bot is disabled
    new Trigger(DriverStation::isEnabled)
        .onTrue(new InstantCommand(()->armAngleSlew.reset(getAbsoluteAngleDegrees())));
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

  public void setArmAngle(double angle) {
    angle = MathUtil.clamp(angle, LOWERSOFTLIMIT, UPPERSOFTLIMIT);

    this.angleSetpoint = angle;

    // one: profiledPid controller -> runs on roborio
    // two: SparkMotionMagic
    // three: Temp solution, slew rate on target value
    // angle = armAngleSlew.calculate(angle);

    double ff = armFF.calculate(getAngle().in(Radians),0);
    armMotor
        .getClosedLoopController()
        .setReference(
            angle,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ff, // TODO: Add me
            ArbFFUnits.kVoltage);
  }

  private double getAbsoluteAngleDegrees() {
    var angle = armMotor.getAbsoluteEncoder().getPosition();
    if(angle > absconversionfactor/4.0){ //Move discontinuity from {-90,90} to {-135, 45}
      angle = angle - absconversionfactor;
    }
    return angle;
  }

  private double getRelativeAngleDegrees(){
    var angle = armMotor.getEncoder().getPosition();
    return angle;
  }


  private double getShooterRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }

  private double getIntakeRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }

  public Angle getAngle(){
    return Degree.of(getAbsoluteAngleDegrees());
  }

  //////////////////////////////////
  /// Define some commands
  /// ///////////////////////////////
 
  public Command testMoveArmWithTrap(DoubleSupplier position){
    return startRun(
      ()->{
        //Seed the initial state/setpoint with the current state
        armSetpoint = new TrapezoidProfile.State(getAbsoluteAngleDegrees(), armMotor.getAbsoluteEncoder().getVelocity());
        // armGoal = new TrapezoidProfile.State(position.getAsDouble(), 0);
      }, 
      ()->{
        //Make sure the goal is dynamically updated
        armGoal = new TrapezoidProfile.State(position.getAsDouble(), 0);

        //update our setpoint to be our next state
        armSetpoint = armTrapProfile.calculate(0.02, armSetpoint, armGoal);
    
        var ff = armFF.calculate(armSetpoint.position, armSetpoint.velocity);
        armMotor.getClosedLoopController()
        .setReference(
          armSetpoint.position,
          ControlType.kPosition, ClosedLoopSlot.kSlot0,
          ff, ArbFFUnits.kVoltage
        );
      }
    );
  }
  
  public Command defaultCommand() {
    return run( () -> {
        setShooterRPM(0);
        setArmAngle(-77);//temporary, so if we pick up algae we dont pop it on the climber
        if (haveAlgae) {
            setIntakeRPM(ROLLERHOLDRPM);
        } else {
            setIntakeRPM(0);
        }
    }).withName("DefaultCommand");
  }


  Trigger isBreakbeamTripped = laserCan.isBreakBeamTripped.debounce((0.03));

  public final double HAVEGRABBEDALGAEAMPS = 50;
  private boolean haveAlgae = false;
  Trigger intakeStalled = new Trigger(()->intakeMotor.getOutputCurrent() > HAVEGRABBEDALGAEAMPS)
    // .and(()->intakeMotor.getEncoder().getVelocity()<200)    
    .debounce(0.1);

  public Command intakeAlgaeFromFloor() {
    return run(() -> {
      setArmAngle(-25);
      setIntakeRPM(ROLLERINTAKERPM);
      setShooterRPM(SHOOTERINTAKERPM);
    })
    .until(intakeStalled).withTimeout(3)
    .andThen(new WaitCommand(0.25))
    .andThen(new InstantCommand(()->{
      intakeMotor.getEncoder().setPosition(0);
      shooterMotor.getEncoder().setPosition(0);
    }))
    .andThen(new RunCommand(()->{
      intakeMotor.getClosedLoopController().setReference(-0.7, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      shooterMotor.getClosedLoopController().setReference(-0.23, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }).withTimeout(1.5))
    .finallyDo((interrupted) -> {
        if (interrupted == false) {
            haveAlgae = true;
        }
    }).withName("IntakeFromFloor");
  }

  public Command prepareToShoot() {
    return prepareToShoot(()->3000);
  }

  public Command prepareToShoot(DoubleSupplier rpm) {
    return run(() -> {
      // setArmAngle(-70);
      setArmAngle(SmartDashboard.getNumber("algae/ShootingAngle", -20));
      setShooterRPM(rpm.getAsDouble());
      setIntakeRPM(ROLLERHOLDRPM);
    }).withName("PrepareToShoot");
  }

  public Command scoreProcessor() {
    return scoreAlgae(() -> -30, () -> 8000)
    .withName("ScoreProcessor");
  }

  public Command shootAlgaeUnchecked(double targetRPM) {
    return run(() -> {
        setShooterRPM(targetRPM);
        // setIntakeRPM(targetRPM*3);//Radius Compensation
        intakeMotor.setVoltage(SmartDashboard.getNumber("algae/ShooterVoltage", 0.8));
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
        prepareToShoot(rpm).withTimeout(3).until(isAtTargetAngle.and(isAtTargetRPM)),
        shootAlgaeUnchecked(rpm.getAsDouble()).withTimeout(1.5)
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

  //Use relative to ensure onboard motor pid is on same page
  public Trigger isAtTargetAngle = new Trigger(() -> {
    return MathUtil.isNear(angleSetpoint, getRelativeAngleDegrees(), ANGLETOLERANCE);
  }).debounce(0.060);

  public Trigger isAtTargetRPM = new Trigger(() -> {
    return MathUtil.isNear(shooterRPMSetpoint, getShooterRPM(), RPMTOLERANCE);
  }).debounce(0.060);

  //TODO:make
  public Trigger needsResync = new Trigger(() -> {
    return false; //absolute encoder is more than 10* off (one tooth) +/-backlash
  })
  .and(()->false) //backlash is either less than one tooth error, or chain tension is accounted for
  .debounce(1); //TODO: if true, try to resync once chain tension is accounted for

  

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
    SmartDashboard.putNumber("algae/arm angle abs", getAbsoluteAngleDegrees());
    // SmartDashboard.putNumber("algae/intake roller pos", intakeMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("algae/shooter roller pos", shooterMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("algae/arm output",armMotor.getOutputCurrent());
    SmartDashboard.putNumber("algae/arm applied output",armMotor.getAppliedOutput());
    SmartDashboard.putNumber("algae/shooter velocity",shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("algae/intake velocity",intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("algae/shooter position",shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("algae/intake position",intakeMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("algae/intake current",intakeMotor.getOutputCurrent());
  }

  AlgaeSim sim = new AlgaeSim(armMotor,intakeMotor,shooterMotor);
  @Override
  public void simulationPeriodic() {
    sim.update();
  }

  AlgaeMech2d mechanism = new AlgaeMech2d(
    this::getAngle,
    ()->RPM.of(getShooterRPM()),
    ()->RPM.of(getIntakeRPM()),
    sim::getAngle
  );

  public Command stop(){
    return run(()->armMotor.stopMotor());
  }

}
