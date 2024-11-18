// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ArcadeDrive;
import frc.robot.Subsystems.DriveSubSystem;

public class RobotContainer {
  private final DriveSubSystem drive = new DriveSubSystem();
  private final XboxController xboxController_0 = new XboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new ArcadeDrive(() -> xboxController_0.getLeftY(), () -> xboxController_0.getLeftX(), drive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
