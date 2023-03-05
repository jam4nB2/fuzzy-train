// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armsub_;


public class setmmmode extends CommandBase {
  private final armsub_ m_arm;
  private final XboxController _driver;
  private double m_targetPos;
 

  /** Creates a new ArmTest. */
  public setmmmode(double targetPos, armsub_ armSubsystem, XboxController _drivecontroller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = armSubsystem; 
    _driver = _drivecontroller;
    m_targetPos = targetPos;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.motionMagicMode(m_targetPos, _driver);
    m_arm.adjustSmoothing(0, 0, _driver);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
