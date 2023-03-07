// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.armsub_;
import frc.robot.commands.toggleMM;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private final armsub_ mArmsub_ = new armsub_();
  private final XboxController _driver = new XboxController(Constants.Xbobcontroller);
  private double m_targetPos;
  
  public RobotContainer() {
      // Configure the trigger bindings
    configureBindings();
    
    double joystickL = -1.0 * _driver.getLeftY();
		double joystickR = -1.0 * _driver.getRawAxis(5); 
    if (Math.abs(joystickL) < 0.10) { joystickL = 0; } /* deadband 10% */
		if (Math.abs(joystickR) < 0.10) { joystickR = 0; } /* deadband 10% */
    JoystickButton dc_rButton = new JoystickButton(_driver, XboxController.Button.kA.value);
    

     dc_rButton.onTrue(new toggleMM(m_targetPos, mArmsub_, _driver));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //}
  }

