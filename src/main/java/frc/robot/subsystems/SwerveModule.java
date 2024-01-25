// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.CANCoderUtil;
import frc.lib.CANSparkMaxUtil;
import frc.lib.OnboardModuleState;
import frc.lib.SwerveModuleConstants;
import frc.lib.CANCoderUtil.CCUsage;
import frc.lib.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
  
    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

   private final SimpleMotorFeedforward feedforward =
   new SimpleMotorFeedforward(
       Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    //creates a feedforward for the swerve drive. feedforward does 90% of the work, estimating stuff
    //PID fixes the error

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.m_angleKP = moduleConstants.angleKP;
        this.m_angleKI = moduleConstants.angleKI;
        this.m_angleKD = moduleConstants.angleKD;
        this.m_angleKFF = moduleConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        //this.?
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(),  getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(),  getAngle()); 
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV supports this now so dont have to worry with rev, but need some funky configs i dont want to do
        //have to be sad with falcons but thats what you get for giving money to Tony
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            //when not taking feedback
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }


    private void setAngle(SwerveModuleState desiredState){
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //the ? and : are a shorthand for an if-else loop
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; 
        
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }




    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition); //may need to change 

      }
    
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }
    
    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        //resets angle motor
        angleMotor.restoreFactoryDefaults();
        //limits can bus usage
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        //sets current limit
        angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        //sets inversion
        angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
        //sets brake mode or not
        angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        //sets a conversion factor for the encoder so it output correlates with the rotation of the module
        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        //oops pid loop time sets the pid
        angleController.setP(m_angleKP);
        angleController.setI(m_angleKI);
        angleController.setD(m_angleKD);
        angleController.setFF(m_angleKFF);
        angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        //burns spark max
        angleMotor.burnFlash();

        Timer.delay(1.0);
        //resets to the cancoder
        resetToAbsolute();
    }

    private void configDriveMotor(){    
        //factory resets the spark max    
        driveMotor.restoreFactoryDefaults();
        //full utilisation on the can loop hell yea
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        //sets current limit
        driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        //sets inverted or not
        driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
        //sets brake mode or not
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        //sets encoder to read velocities as meters per second
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        //sets encoder to read positions as meters traveled
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
        //pid setting fun
        driveController.setP(Constants.SwerveConstants.driveKP);
        driveController.setI(Constants.SwerveConstants.driveKI);
        driveController.setD(Constants.SwerveConstants.driveKD);
        driveController.setFF(Constants.SwerveConstants.driveKFF);
        driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        //burns to spark max
        driveMotor.burnFlash();
        //resets encoder position to 0
        driveEncoder.setPosition(0.0);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }
    









  

}
