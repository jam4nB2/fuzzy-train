// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.Constants;


public class armsub_ extends SubsystemBase {

    private final WPI_TalonFX _outerArmLeft = new WPI_TalonFX(Constants.ArmProfile.LEFT_OUTER_ARM);
    private final WPI_TalonFX _outerArmRight = new WPI_TalonFX(Constants.ArmProfile.RIGHT_OUTER_ARM);
    private final WPI_TalonSRX _innerArmLeft = new WPI_TalonSRX(Constants.ArmProfile.LEFT_INNER_ARM);
    private final WPI_TalonSRX _innerArmRight = new WPI_TalonSRX(Constants.ArmProfile.RIGHT_INNER_ARM);
    
    int _pov = -1;
    int _smoothing = 0; 

  public armsub_() {
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
    
  }
    
  public void motionMagicMode(double targetPos, XboxController _driver){
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
    }

    _pov = pov;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
  
  

