// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubSystem;

public class ArcadeDrivePID extends Command {
  private DoubleSupplier vel;
  private DoubleSupplier rot;
  private DriveSubSystem drive;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrivePID(DoubleSupplier velocity, DoubleSupplier rotation, DriveSubSystem drive) 
  {
   this.vel=velocity;
   this.rot=rotation;
   this.drive=drive;
   addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrivePID(vel.getAsDouble(), rot.getAsDouble(),0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
