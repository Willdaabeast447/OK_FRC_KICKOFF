// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ArcadeDrivePID;
import frc.robot.Commands.Test1;
import frc.robot.Subsystems.DriveSubSystem;

public class RobotContainer {
  private final DriveSubSystem drive= new DriveSubSystem();
  private final XboxController xboxController_1= new XboxController(0);
  private final SendableChooser<Command> autoChooser;
  private final Command test=new Test1();

 
  public RobotContainer() {
    NamedCommands.registerCommand("autoBalance",test);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();

  }

  private void configureBindings() {
    drive.setDefaultCommand(new ArcadeDrivePID(()->xboxController_1.getLeftY(),()-> xboxController_1.getLeftX(), drive));
  }

public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
