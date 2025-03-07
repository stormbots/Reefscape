// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

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
import com.stormbots.LaserCanWrapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

  public static final double ROLLERINTAKERPM = 3500;

  private static final double ANGLETOLERANCE = 5;

  private static final double RPMTOLERANCE = 200;

  private double angleSetpoint = -90;
  private double shooterRPMSetpoint = 0;
  private double intakeRPMSetpoint = 0;

  //TODO: Find real angles
  private ArmFeedforward armFF = new ArmFeedforward(0.3, 0.25, 0);


  private final TrapezoidProfile armTrapProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(30, 30)
  );

  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();

  private boolean hasSynced = false;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {
    // this.io = io;

    armMotor.configure(AlgaeGrabberConfigs.getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(AlgaeGrabberConfigs.getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor.configure(AlgaeGrabberConfigs.getShooterConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //MUST BE BETWEEN [-135,45] WHEN INITIALIZED, syncs encoders
    Timer.delay(0.1);
    armMotor.getEncoder().setPosition(getAbsoluteAngleDegrees());

    SmartDashboard.putNumber("algae/ShootingAngle", -10);
    SmartDashboard.putNumber("algae/IntakeVoltage", 0.8);
    //Automatically reset the slew rate if the bot is disabled
    new Trigger(DriverStation::isEnabled)
        .onTrue(new InstantCommand(()->armAngleSlew.reset(getAbsoluteAngleDegrees())))
        //Syncs on first enable, redundancy, sometimes doesn't sync on startup
        .and(()->!hasSynced)
        .onTrue(new InstantCommand(()->{
          armMotor.getEncoder().setPosition(getAbsoluteAngleDegrees());
          hasSynced = true;
        }));


    setDefaultCommand(stow().repeatedly()); //TODO enable me

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
    this.intakeRPMSetpoint = rpm;
    intakeMotor
        .getClosedLoopController()
        .setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
  }

  public void setArmAngle(double angle) {
    angle = MathUtil.clamp(angle, AlgaeGrabberConfigs.kLowerSoftLimit, AlgaeGrabberConfigs.kUpperSoftLimit);

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
    if(angle > AlgaeGrabberConfigs.kAbsEncoderConversionFactor/4.0){ //Move discontinuity from {-90,90} to {-135, 45}
      angle = angle - AlgaeGrabberConfigs.kAbsEncoderConversionFactor;
    }
    return angle;
  }

  private double getAngleDegrees(){
    var angle = armMotor.getEncoder().getPosition();
    return angle;
  }


  private double getShooterRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }

  private double getIntakeRPM() {
    return intakeMotor.getEncoder().getVelocity();
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


  Trigger isBreakbeamTripped = laserCan.isBreakBeamTripped.debounce((0.03))
  .or(()->Timer.getFPGATimestamp()%6<1);

  Trigger intakeStalled = new Trigger(()->intakeMotor.getOutputCurrent() > 50)
    .and(()->intakeMotor.getEncoder().getVelocity()<250)    
    .debounce(0.1);


  public Command newIntakeFromGround(){
    return new SequentialCommandGroup(
      run(() ->{
        setArmAngle(-29);
        setIntakeRPM(3500);
        shooterMotor.stopMotor();
      }).until(isBreakbeamTripped)
      //todo: Do we need to have a settling operation to ensure the encoder reset is reliable?
    )
    .finallyDo(()->intakeMotor.getEncoder().setPosition(0))
    .withName("IntakeAlgae")
    ;
  };

  public Command newIntakeFromElevator(){
    //TODO: Needs implementation
    return new SequentialCommandGroup(
    );
  }

  public Command stow(){ //TODO set to default
    //if havealgae
      //then turn ofmotors, go to hold  angle
    //if no algae
      // angle down
      //run motors to loadfrom stuck inside  bot
    //go to resting angle, motors off or hold position
    var stowangleEmpty = AlgaeGrabberConfigs.kLowerSoftLimit;
    var stowangleAlgae = -70;

    var withAlgae = run(()->{
      shooterMotor.stopMotor();
      setArmAngle(stowangleAlgae);
      setIntakeRPM(0);
    }).onlyWhile(isBreakbeamTripped);

    var withoutAlgae = new SequentialCommandGroup(
      run(()->{
        shooterMotor.stopMotor();
        setIntakeRPM(ROLLERINTAKERPM);
        setArmAngle(stowangleEmpty);
      }).withTimeout(2),
      run(()->{
        intakeMotor.stopMotor();
        shooterMotor.stopMotor();
        setArmAngle(stowangleEmpty);
      })
    ).until(isBreakbeamTripped)
    ;

    return new ConditionalCommand(
      withAlgae,
      withoutAlgae,
      isBreakbeamTripped
      ).withName("StowAlgae");
  }

  public Command newShootAlgae(){
    double angle = -25;
    double shooterrpm = 4000;
    return new SequentialCommandGroup(
      clearShooterForShooting(),
      new WaitCommand(0.1),
      //run prepare operation so we're at the right angle
      run(() ->{
        //Leaving intake at holding PID from clearShooter
        setArmAngle(angle);
        setShooterRPM(shooterrpm);
      }).until(isAtTargetAngle.and(isAtTargetRPM)),
      new WaitCommand(0.1),
      //Actually shoot things
      run(() ->{
        setArmAngle(angle);
        setIntakeRPM(shooterrpm);
        setShooterRPM(shooterrpm);
      })
    ).withName("ShootAlgae")
    ;
  };


  private Command clearShooterForShooting(){
    //assume have algae
    //run intake motors back for set time
    //check current/speed on shooter?
    //grapple? Probably not viable or consistent
    //run back X rotations

    var intakeClearance = -0.7;
    var shooterClearance = -0.23;
    //Brian solution was this: 
    return new SequentialCommandGroup(
        new RunCommand(()->{
          intakeMotor.getClosedLoopController().setReference(intakeClearance, ControlType.kPosition, ClosedLoopSlot.kSlot1);
          shooterMotor.getClosedLoopController().setReference(shooterClearance, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        },this)
        .until(()->MathUtil.isNear(intakeClearance, intakeMotor.getEncoder().getPosition(), 0.1))
        .withTimeout(0.5)
      );
    //TODO: We can't have it rely on a long timer. 
    //Validate that a process like this works reliably
  }

  public Command newScoreProcessor(){
    var rpm=4000;
    return run(()->{
      armMotor.stopMotor();
      setIntakeRPM(rpm);
      setShooterRPM(rpm);
    }).withName("ScoreProcessor");
  };


  //TODO: If this is a needed panic sequence, name it appropriately 
  // public Command ejectFromFloor(){
  //   return run(()->{
  //     setArmAngle(-5);
  //     setIntakeRPM(-ROLLERINTAKERPM); //temp
  //     // setShooterRPM(SHOOTERINTAKERPM);
  //     shooterMotor.setVoltage(-1.0);
  //   });
  // }

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
    return MathUtil.isNear(angleSetpoint, getAngleDegrees(), ANGLETOLERANCE);
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
    SmartDashboard.putNumber("algae/SHOOTER current",shooterMotor.getOutputCurrent());
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
    ()->RPM.of(getIntakeRPM()),
    ()->RPM.of(getShooterRPM()),
    isBreakbeamTripped,
    sim::getAngle
  );

  public Command stop(){
    return runOnce(()->{
      armMotor.stopMotor();
      intakeMotor.stopMotor();
      shooterMotor.stopMotor();
    })
    .andThen(Commands.idle(this))
    ;
  }

}
