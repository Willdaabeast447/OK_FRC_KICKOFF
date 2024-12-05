// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.DriveSubSystem;

public class RobotContainer {
  private final DriveSubSystem drive= new DriveSubSystem();
  private final XboxController xboxController_1= new XboxController(0);
  private Command jukeboxhero =drive.DriveWithGuitarPID(()->xboxController_1.getPOV(),
                                                      ()->xboxController_1.getRawButton(1), 
                                                      ()->xboxController_1.getRawButton(2),
                                                       ()->xboxController_1.getRightX());
 
  public RobotContainer() {
   
    configureBindings();

  }

  private void configureBindings() {
    drive.setDefaultCommand(jukeboxhero);
  }

public Command getAutonomousCommand() {
    return null;
  }
}
