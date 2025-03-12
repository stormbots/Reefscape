package frc.robot.subsystems.AlgaeGrabber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class AlgaeGrabberConfigs {
    
    public static final double kAbsEncoderConversionFactor = 360/2.0;
    public static final double kLowerSoftLimit = -100;
    public static final double kUpperSoftLimit = 10;
  
  
    public static SparkBaseConfig getArmConfig(){
        var armConf = new SparkFlexConfig()
        .inverted(false)
        .smartCurrentLimit(30)
        .closedLoopRampRate(.2)
        .idleMode(IdleMode.kBrake)
        ;

        armConf.absoluteEncoder
            .positionConversionFactor(kAbsEncoderConversionFactor)//This is on a 2:1 gear step
            .velocityConversionFactor(kAbsEncoderConversionFactor / 60.0)
            .inverted(false);
        var conversionfactor=360.0/(45*2.0);//45:1 planetary reduction, 2:1 sprocket
        armConf.encoder
        .positionConversionFactor(conversionfactor)
        .velocityConversionFactor(conversionfactor/60.0);
        ;
        armConf.softLimit
        .forwardSoftLimit(kUpperSoftLimit).forwardSoftLimitEnabled(false)
        .reverseSoftLimit(kLowerSoftLimit).reverseSoftLimitEnabled(false)
        
        ;

        armConf.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.2*4/60.0)
        .i((12/50)/20)
        .iZone(10)
        ;

        return armConf;
    }


    public static SparkBaseConfig getIntakeConfig(){
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
        .p(0.1*4*2*2*2*2*1.5/9, ClosedLoopSlot.kSlot1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        


        rollerConf.closedLoopRampRate(0.05);
        rollerConf.openLoopRampRate(0.05);
        return rollerConf;
    }

    public static SparkBaseConfig getShooterConfig(){
        var shooterConf = new SparkFlexConfig();
        shooterConf
        .inverted(false)
        .smartCurrentLimit(80) //TODO: Test Code
        .idleMode(IdleMode.kBrake)
        ;

        // shooterConf.encoder
        // .uvwMeasurementPeriod(8)
        // .uvwAverageDepth(2)
        // .quadratureMeasurementPeriod(2)
        // .quadratureAverageDepth(2);

        shooterConf.closedLoop
        // .p(1/500.0)
        .velocityFF(1/5760.0*0.95)
        .p(0.1*4*2*2*2*2*1.5/3, ClosedLoopSlot.kSlot1)
        .i(5/50)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        shooterConf.closedLoopRampRate(0.05);
        shooterConf.openLoopRampRate(0.05);

        return shooterConf;
    }


}
