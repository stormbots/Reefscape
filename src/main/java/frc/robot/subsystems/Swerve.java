// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldNavigation;
import frc.robot.subsystems.Scorer.Scorer;
import swervelib.SwerveDrive;
import swervelib.imu.NavXSwerve;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

  public Field2d odometryField = new Field2d();

  
  double maximumSpeed = 5.033;

  SwerveDrive swerveDrive;

  PathConstraints constraintsFast = new PathConstraints(5, 3, 5, 3);
  PathConstraints constraintsSlow = new PathConstraints(2.5, 1.75, 5, 1.5);

  //2.24-1.88
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 2.75, 0.35);

  /** Creates a new SwerveSubsystem. */
  public Swerve() {

    SmartDashboard.putData("odometryField", odometryField);
    
    //Need to turn this back on when running path, commented out for now because its angry
    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"skipper");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }  

    enableKA(true);
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));

    swerveDrive.setMotorIdleMode(true); //just to be safe

    configurePathplanner();
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    // PathfindingCommand.warmupCommand();

    new Trigger(DriverStation::isTeleopEnabled).onTrue(new InstantCommand(()->
      enableKA(false)
    ));

    new Trigger(DriverStation::isAutonomousEnabled).onTrue(new InstantCommand(()->
      enableKA(true)
    ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
    odometryField.setRobotPose(swerveDrive.getPose());

    SmartDashboard.putNumber("serve/wheelencoder", swerveDrive.getModules()[0].getPosition().distanceMeters);
    
    SmartDashboard.putNumber("swerve/heading", swerveDrive.getOdometryHeading().getDegrees());
    Logger.recordOutput("swerve/SwerveModuleStates", swerveDrive.getStates());
    Logger.recordOutput("swerve/pose", swerveDrive.getPose());
    SmartDashboard.putNumber("swerve/x", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("swerve/y", swerveDrive.getPose().getY());
    
    SmartDashboard.putBoolean("Navx/Connected", isGyroConnected());
    SmartDashboard.putBoolean("Navx/Callibrating", isGyroCallibrating());

    SmartDashboard.putBoolean("barge/BargeOKRed", isWithinShootingRangeRed.getAsBoolean());
    SmartDashboard.putBoolean("barge/BargeOKBlue", isWithinShootingRangeBlue.getAsBoolean());
    SmartDashboard.putBoolean("barge/BargeOKField", isWithinShootingRange.getAsBoolean());

  }

  public void configurePathplanner(){
    RobotConfig robotConfig = null;

    // double kModuleXOffset = Meters.convertFrom(23.5, Inches)/2.0;
    // double kModuleYOffset = Meters.convertFrom(23.5, Inches)/2.0;


    // Translation2d[] moduleOffsets = new Translation2d[]{
    //   new Translation2d(kModuleXOffset, kModuleYOffset),
    //   new Translation2d(kModuleXOffset, -kModuleYOffset),
    //   new Translation2d(-kModuleXOffset, kModuleYOffset),
    //   new Translation2d(-kModuleXOffset, -kModuleYOffset)
    // };

    // ModuleConfig moduleConfig = new ModuleConfig(
    //   Meters.convertFrom(1.5, Inches), 
    //   5.1, 
    //   1.3, 
    //   DCMotor.getNeoVortex(1).withReduction(5.076923076923077), 
    //   40, 
    //   1);

    // robotConfig = new RobotConfig(
    //   Pounds.of(30), 
    //   KilogramSquareMeters.of(15), 
    //   moduleConfig, 
    //   moduleOffsets
    // );

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // TODO: handle exception
    }

    BooleanSupplier shouldFlipPath = () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getChassisSpeeds,
      this::setChassisSpeeds, 
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.5, 0.0, 0.0) // Rotation PID constants
      ), 
      robotConfig, 
      shouldFlipPath, 
      this
    );

  
  }

  public Command driveCommandAllianceManaged(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    BooleanSupplier isRed = () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    return run(() -> {
      // Make the robot move
      if(isRed.getAsBoolean()){
        swerveDrive.drive(new Translation2d(-translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      }
      else{
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      }
    });
  }

  private Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public boolean shouldFlipPoseBasedOnAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  //Robot Relative version
  /**
   * 
   * @param translationX
   * @param translationY 
   * @param angularRotationX Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot relativity.
   * @return
   */
  public Command driveCommandRobotRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  //Aligns to a heading from the DRIVERS perspective
  //Doesnt wrap or anything, literally the worst functionality. IDK why i made life hard for myself
  public Command alignToCoralStation(DoubleSupplier translationX, DoubleSupplier translationY, Rotation2d desiredRotation){
    
    //TODO Get nearest coral station tag
    // Get angle of that tag
    // Align to that angle
    
    // PIDController pid = new PIDController(2.0/360, 0.0, 0.0);
    ProfiledPIDController commandTurnToPid = new ProfiledPIDController(2.0/360.0, 0, 0, new TrapezoidProfile.Constraints(720, 540));
    commandTurnToPid.enableContinuousInput(0, 360);

    // commandTurnToPid.reset //TODO when we touch the pid we need to do this
    BooleanSupplier shouldFlip = ()->{
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
    
    return run(()->{
      double sign = 1.0;
      double desiredRotationDegrees = desiredRotation.getDegrees();
      //Flips across y=x
      if(shouldFlip.getAsBoolean()){
        sign=-1.0;
        desiredRotationDegrees-=180;
        if(desiredRotationDegrees<180){
          desiredRotationDegrees+=360;
        }
      }
      
      double power = commandTurnToPid.calculate(getPose().getRotation().getDegrees(), desiredRotationDegrees);
      swerveDrive.drive(new Translation2d(sign*translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          sign*translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        power * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public Rotation2d getHeading(){
    return swerveDrive.getGyro().getRotation3d().toRotation2d();
  }

  public Command goToPose(Pose2d targetPose){

    return run(()->{
      //get current pose
      var pose = getPose();
      var delta = targetPose.relativeTo(pose);  

      var distancex_m = delta.getMeasureX().in(Units.Meter); // the distance in the x axis
      var distancey_m = delta.getMeasureY().in(Units.Meter); //distnace in y
      var rotation_d = delta.getRotation().getDegrees();

      //move -> robot relative
      swerveDrive.drive(new Translation2d(distancex_m * swerveDrive.getMaximumChassisVelocity(),
                                          distancey_m * swerveDrive.getMaximumChassisVelocity()),
                        (rotation_d/360.0) * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  //does not reset based off field
  private void pidToPoseFast(Pose2d pose){
    final double transltionP = 3.0*1.2;
    final double thetaP = 2.0*4*1.2 ;

    double clamp = 2.0;

    Pose2d delta = pose.relativeTo(swerveDrive.getPose());

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(
      MathUtil.clamp(delta.getX()*transltionP,-clamp, clamp),
      MathUtil.clamp(delta.getY()*transltionP,-clamp,clamp),
      delta.getRotation().getRadians()*thetaP
    ));

    odometryField.getObject("pidtarget").setPose(pose);
    SmartDashboard.putNumber("swerve/pid/deltax", pose.getX());
    SmartDashboard.putNumber("swerve/pid/deltay", pose.getY());

  }

  private void pidToPosePrecise(Pose2d pose){
    final double transltionP = 3.0*1.2*1.5*1.5*1.5;
    final double thetaP = 2.0*4*1.2;
    

    double clamp = 0.75;

    Pose2d delta = pose.relativeTo(swerveDrive.getPose());

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(
      MathUtil.clamp(delta.getX()*transltionP,-clamp, clamp),
      MathUtil.clamp(delta.getY()*transltionP,-clamp,clamp),
      delta.getRotation().getRadians()*thetaP
    ));

    SmartDashboard.putNumber("swerve/pidTargetPoseX", pose.getX());
    SmartDashboard.putNumber("swerve/pidTargetPoseY", pose.getY());

  }

  public Command pidToPoseFastCommand(Pose2d poseSupplier){
    return run(()->pidToPoseFast(poseSupplier)).finallyDo(()->stop());
  }

  public Command pidToPosePreciseCommand(Pose2d poseSupplier){
    return run(()->pidToPosePrecise(poseSupplier));
  }

  private Pose2d flipPoseIfAppropriate(Pose2d pose){
    if(shouldFlipPoseBasedOnAlliance()){
      //return new Pose2d(17.55-pose.getX(),8.05-pose.getY(), pose.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));
      return FlippingUtil.flipFieldPose(pose); //We should just use this
    }
    return pose;
  }

  Trigger isWithinShootingRangeRed = new Trigger(()-> {
    return isNearEnoughShoot(11.713);
  });
  Trigger isWithinShootingRangeBlue = new Trigger(()-> {
    return isNearEnoughShoot(5.835);
  });
  public Trigger isWithinShootingRange = isWithinShootingRangeRed.or(isWithinShootingRangeBlue);

  public boolean isNearEnoughShoot(double x){
    var target = new Pose2d(x, swerveDrive.getPose().getY(), new Rotation2d(0.0));
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(10).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnoughToShoot",result);
    return result;
  }

  public boolean isNearEnoughToPID(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(35).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnough",result);
    return result;
  }

  public boolean isNearEnoughToPIDAuto(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(5.0).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnough",result);
    return result;
  }

  public boolean isNearEnoughToPIDPrecise(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(48).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnough",result);
    return result;
  }

  public boolean isNearEnoughToScore(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    return distance <= Inches.of(1).in(Meters);
  }

  public boolean isNear(Pose2d target,double distance){
    var delta = target.getTranslation().getDistance(getPose().getTranslation());
    return delta <= Inches.of(distance).in(Meters);
  }
  //Now uses pathfind, not compatible with previous
  private Command privatePathToPose(Pose2d pose){
    return Commands.sequence(
      pidToPoseFastCommand(pose)
      .until(()->isNearEnoughToPID(pose))
      .withTimeout(5),
      pidToPosePreciseCommand(pose).until(()->isNearEnoughToScore(pose)).withTimeout(3.0),
      new InstantCommand(this::stop,this)
    );
  }
  private Command privatePathToPoseAuto(Pose2d pose){
    return Commands.sequence(
      new InstantCommand(()->enableKA(true)),
      AutoBuilder.pathfindToPose(FieldNavigation.getCoralMid(pose).transformBy(new Transform2d(Units.Inches.of(15.0).in(Meters),0.0,new Rotation2d())), constraintsFast)
      .until(()->isNearEnoughToPIDAuto(pose))
      .withTimeout(5),
      pidToPosePreciseCommand(pose).until(()->isNearEnoughToScore(pose)).withTimeout(3.0),
      new InstantCommand(this::stop,this)
    ).finallyDo(()->enableKA(false));
  }
  

  private Command privatePathToOffset(Pose2d pose){
    // return pidToPoseCommand(pose).until(()->isNear(pose, 12.0)).withTimeout(4.5);
    return pidToPoseFastCommand(pose)
      .until(()->isNear(pose, 12.0))
      .withTimeout(4.5)
    ;
  }

  private Command privatePathToPosePrecise(Pose2d pose){
    return Commands.sequence(
      pidToPoseFastCommand(pose).until(()->isNearEnoughToPIDPrecise(pose)).withTimeout(4.5),
      pidToPosePreciseCommand(pose).until(()->isNearEnoughToScore(pose)).withTimeout(1.5),
      new InstantCommand(this::stop,this)
    );
  }
  // public Command followPath(PathPlannerPath path)
  // {
  //   return AutoBuilder.followPath(path);
  // }

  public Command pidToCoralLeft(){
    return new DeferredCommand(()->pidToPoseFastCommand(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pidToCoralRight(){
    return new DeferredCommand(()->pidToPoseFastCommand(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command pidToCoralLeftHuman(){
    return new DeferredCommand(()->privatePathToPosePrecise(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pidToCoralRightHuman(){
    return new DeferredCommand(()->privatePathToPosePrecise(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command pidToCoralSource(){
    return new DeferredCommand(()->pidToPoseFastCommand(FieldNavigation.getCoralSource(getPose())), Set.of(this));
  }

  public Command pathToCoralLeft(){
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pathToCoralRight(){
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }
  public Command pathToCoralLeftAuto(){
    return new DeferredCommand(()->privatePathToPoseAuto(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pathToCoralRightAuto(){
    return new DeferredCommand(()->privatePathToPoseAuto(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command pathToCoralSource(){
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralSource(getPose())), Set.of(this));
  }
  public Command pathToCoralSourceAuto(){
    return new DeferredCommand(()->privatePathToPoseAuto(FieldNavigation.getCoralSource(getPose())), Set.of(this));
  }

  public Command pathToOffsetLeft(){
    return new DeferredCommand(()->privatePathToOffset(FieldNavigation.getOffsetCoralLeft(getPose())), Set.of(this));
  }

  public Command pathToOffsetRight(){
    return new DeferredCommand(()->privatePathToOffset(FieldNavigation.getOffsetCoralRight(getPose())), Set.of(this));
  }

  public Command pathToReefAlgae(){
    Set<Subsystem> set = Set.of(this);
    return new DeferredCommand(()->privatePathToPosePrecise(FieldNavigation.getReefAlgae(getPose())),set);
  }
  


  public Command pathToPath(PathPlannerPath targetPath){
    //robot constraints for pathPlanner
    PathConstraints constraints = new PathConstraints(5, 3.5, 5, 3);

    //Go to target Path and follow it
    return AutoBuilder.pathfindThenFollowPath(targetPath, constraints);
  }

  public boolean isGyroConnected(){
    var navx = (AHRS)swerveDrive.getGyro().getIMU();
    if(navx.isConnected()){
      return true;
    }
    return false;
  }
  public boolean isGyroCallibrating(){
    var navx = (AHRS)swerveDrive.getGyro().getIMU();
    if(navx.isCalibrating()){
      return true;
    }
    return false;
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  private void resetOdometry(Pose2d initialHolonomicPose){
    System.out.print("RESETTING ODOMETRY");
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  @Deprecated //We just set the position from the initial pose
  public void resetOdometryAllianceManaged(Pose2d initialHolonomicPose){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Red){
      //Flips across y=x, not across vertical acis
      initialHolonomicPose  = new Pose2d(17.55-initialHolonomicPose.getX(),8.05-initialHolonomicPose.getY(), initialHolonomicPose.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));
    }
    
    resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  // do we need the function based version?
   public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
     swerveDrive.setChassisSpeeds(chassisSpeeds);
   }
  
  public Command resetGyro(){
    return runOnce(swerveDrive::zeroGyro).ignoringDisable(true);
  }

  public Command stopCommand(){
    return run(this::stop);
  }
  private void stop(){
    swerveDrive.setChassisSpeeds(new ChassisSpeeds());
  }

  /** Spam path data to the field for validation */
  public void plotPathOnField(PathPlannerPath path, Field2d field){
    if(DriverStation.isFMSAttached())return;  //Don't do this at comps
    // if(Robot.isReal())return;  //Don't do this on a real bot

    field.getObject("path").setPoses(
      path.getPathPoses()
      .stream()
      .map(this::flipPoseIfAppropriate)
      .toList()
    );
  }

  public void setInitialPoseFromPath(String name){
    
    try {
      var path = PathPlannerPath.fromPathFile(name);
      var poseOptional = path.getStartingHolonomicPose();
      if(poseOptional.isEmpty()) return ;
      var pose = poseOptional.get();
      pose = flipPoseIfAppropriate(pose);

      plotPathOnField(path, odometryField);

      resetOdometry(pose);

    } catch (Exception e) {
    }
  }

  public Command followPath(String name){
    try {
      var path = PathPlannerPath.fromPathFile(name);

      plotPathOnField(path, odometryField);

      return AutoBuilder.followPath(path).finallyDo(this::stop);
    } catch (Exception e) {
      // TODO: handle exception
      for(int i=0; i<1000; i++){
        System.out.println(e);
      }
      return new InstantCommand();
    }
  }

  public void enableKA(boolean enabled){
    if(enabled){
      swerveDrive.replaceSwerveModuleFeedforward(feedforward);
    }
    else{
      swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(feedforward.getKs(), feedforward.getKv(), 0.0));
    }
  }
}
