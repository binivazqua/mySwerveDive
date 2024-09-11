// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
import frc.robot.subsystems.swerveDrive.swerveModule;

public class RobotContainer {
  private SwerveSubsystem mySwerve;
  private Joystick driverControl;

  public RobotContainer() {

    mySwerve = SwerveSubsystem.getInstance();
    driverControl = new Joystick(0);
    //mySwerve = new SwerveSubsystem(driverControl);


      
    mySwerve.setDefaultCommand(new SwerveDriveCmd(
      () -> driverControl.getRawAxis(1),
      () -> driverControl.getRawAxis(0),
      () -> driverControl.getRawAxis(2),
      () -> { 
        boolean IFO = false;
        if (driverControl.getRawButton(1)){
          IFO = true;
        } else if (driverControl.getRawButton(2)){
          IFO = false;  
        }

        return IFO;
        
      }
    )); 
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
