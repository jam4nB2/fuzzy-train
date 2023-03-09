// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;


public class armsub_ extends SubsystemBase {

    private final WPI_TalonFX _outerArmLeft;
    private final WPI_TalonFX _outerArmRight;
    private final WPI_TalonSRX _innerArmLeft;
    private final WPI_TalonSRX _innerArmRight;
    private XboxController m_Controller;

    double targetPos;
    int _pov = -1;
    int _smoothing = 0; 
    int leftY;
    double _driver;
    int kMeasuredPosHorizontal = -1000; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation

  public armsub_() {
    _outerArmLeft = new WPI_TalonFX(Constants.ArmProfile.LEFT_OUTER_ARM);
    _outerArmRight = new WPI_TalonFX(Constants.ArmProfile.RIGHT_OUTER_ARM);
    _innerArmLeft = new WPI_TalonSRX(Constants.ArmProfile.LEFT_INNER_ARM);
    _innerArmRight = new WPI_TalonSRX(Constants.ArmProfile.RIGHT_INNER_ARM);

    configureOuter();
    configureInner();

  }


    public void configureOuter(){
      _outerArmLeft.configFactoryDefault();
    _outerArmLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    _outerArmLeft.configNeutralDeadband(0.04, Constants.kTimeoutMs); // .04 is default deadband
    _outerArmLeft.setSensorPhase(false);
    _outerArmLeft.setInverted(false);
    _outerArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    _outerArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    
    _outerArmLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		_outerArmLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_outerArmLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
		_outerArmLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
 
    _outerArmLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    _outerArmLeft.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_outerArmLeft.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_outerArmLeft.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_outerArmLeft.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
 
    _outerArmLeft.configMotionCruiseVelocity(Constants.ArmProfile.CRUISE_VEL, Constants.kTimeoutMs);
    _outerArmLeft.configMotionAcceleration(Constants.ArmProfile.MOTION_ACCEL, Constants.kTimeoutMs);
 
    _outerArmLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    _outerArmRight.configFactoryDefault();
    _outerArmRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    _outerArmRight.configNeutralDeadband(0.04, Constants.kTimeoutMs); // .04 is default deadband
    _outerArmRight.setSensorPhase(false);
    _outerArmRight.setInverted(false);
    _outerArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    _outerArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    
    _outerArmRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		_outerArmRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_outerArmRight.configPeakOutputForward(1, Constants.kTimeoutMs);
		_outerArmRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);
 
    _outerArmRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    _outerArmRight.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_outerArmRight.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_outerArmRight.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_outerArmRight.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
 
    _outerArmRight.configMotionCruiseVelocity(Constants.ArmProfile.CRUISE_VEL, Constants.kTimeoutMs);
    _outerArmRight.configMotionAcceleration(Constants.ArmProfile.MOTION_ACCEL, Constants.kTimeoutMs);
 
    _outerArmRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
      

    public void configureInner(){
      /* Inner Arm Left */
      _innerArmLeft.configFactoryDefault();
      _innerArmLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
      
      _innerArmLeft.configNeutralDeadband(0.04, Constants.kTimeoutMs); // .04 is default deadband
      _innerArmLeft.setSensorPhase(false);
      _innerArmLeft.setInverted(false);
      _innerArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
      _innerArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
      
      _innerArmLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
      _innerArmLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
      _innerArmLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
      _innerArmLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
   
      _innerArmLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
      _innerArmLeft.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
      _innerArmLeft.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
      _innerArmLeft.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
      _innerArmLeft.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
   
      _innerArmLeft.configMotionCruiseVelocity(Constants.ArmProfile.CRUISE_VEL, Constants.kTimeoutMs);
      _innerArmLeft.configMotionAcceleration(Constants.ArmProfile.MOTION_ACCEL, Constants.kTimeoutMs);
   
      _innerArmLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  
      /* Inner Arm Right */
      _innerArmRight.configFactoryDefault();
      _innerArmRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
      
      _innerArmRight.configNeutralDeadband(0.04, Constants.kTimeoutMs); // .04 is default deadband
      _innerArmRight.setSensorPhase(false);
      _innerArmRight.setInverted(false);
      _innerArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
      _innerArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
      
      _innerArmRight.configNominalOutputForward(0, Constants.kTimeoutMs);
      _innerArmRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
      _innerArmRight.configPeakOutputForward(1, Constants.kTimeoutMs);
      _innerArmRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);
   
      _innerArmRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
      _innerArmRight.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
      _innerArmRight.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
      _innerArmRight.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
      _innerArmRight.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
     
      _innerArmRight.configMotionCruiseVelocity(Constants.ArmProfile.CRUISE_VEL, Constants.kTimeoutMs);
      _innerArmRight.configMotionAcceleration(Constants.ArmProfile.MOTION_ACCEL, Constants.kTimeoutMs);
   
      _innerArmRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    //Set control mode to Magic Motion if A is pressed 
  public void setMagicMotionOuter(double targetPos, XboxController _driver){
    double leftY = -1.0 * _driver.getLeftY(); 
    double rightY = -1.0 * _driver.getRawAxis(5); 

    if (_driver.getAButtonPressed()){

      targetPos = leftY * 2048 * 10.0;
      
      _outerArmLeft.set(TalonFXControlMode.MotionMagic, targetPos);
  }
    else {
      _outerArmLeft.set(TalonFXControlMode.PercentOutput,leftY);

    }
}

public void setMagicMotionInner(double targetPos, XboxController _driver){
  double rightY = -1.0 * _driver.getRawAxis(5); 
  if (_driver.getBButton()){
      targetPos = rightY * 2048 * 10.0;

      _innerArmLeft.set(TalonSRXControlMode.PercentOutput, targetPos, DemandType.ArbitraryFeedForward, rightY);
  }
}

public void zeroSelectedFeedbackSensor(){
  _outerArmLeft.setSelectedSensorPosition(0);
}

public void adjustSmoothing(int _smoothing, int _pov, XboxController _driver){
  int pov = _driver.getPOV();
  if (_pov == pov) {

  }
    else if (_pov == 180) {
      _smoothing--;
      if (_smoothing < 0)
          _smoothing = 0;

        _outerArmLeft.configMotionSCurveStrength(_smoothing);
        _innerArmLeft.configMotionSCurveStrength(_smoothing);

    }
      else if (_pov == 0) { // D-Pad up
        _smoothing++;
        if (_smoothing > 8)
      _smoothing = 8;
    _outerArmLeft.configMotionSCurveStrength(_smoothing);
    _innerArmLeft.configMotionSCurveStrength(_smoothing);
    _pov = pov;
  }
}

public void adjustOuterArm(){
  _outerArmLeft.set(TalonFXControlMode.MotionMagic, _driver);
  adjustSmoothing(_smoothing, _pov, m_Controller);
}

public void adjustInnerArm(){
  _innerArmLeft.set(TalonSRXControlMode.MotionMagic, _driver);
  adjustSmoothing(_smoothing, _pov, m_Controller);
}

public void calculateGravityOffset(){
double currentPos = _innerArmLeft.getSelectedSensorPosition();
double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
double radians = java.lang.Math.toRadians(degrees);
double cosineScalar = java.lang.Math.cos(radians);

double maxGravityFF = 0.07;
_innerArmLeft.set(TalonSRXControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
  
  

