package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
import frc.robot.utilities.myConstants;
import frc.robot.utilities.myConstants.SwerveSubsystemKs;

import java.util.function.Supplier;

public class SwerveDriveCmd extends Command {
    private final SwerveSubsystem swerveDrive;
    private final Supplier<Double> driveSupp, strafeSupp, turnSupp;
    private final Supplier<Boolean> isFieldOriented;
    private final SlewRateLimiter driveSpeedLimiter, strafeSpeedLimiter, turnSpeedLimiter;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param driveSupp A double supplier that represents the value of drive velocity.
   * @param turnSupp A double supplier that represents the value of turn velocity.
   * @param strafeSupp A double supplier that represents the value of strafe velocity.
   * 
   */

   /*
    * Supplier<type>  as our velocities are variable.
    */
  public SwerveDriveCmd(
  Supplier<Double> driveSupp, 
  Supplier<Double> strafeSupp, 
  Supplier<Double> turnSupp, 
  Supplier<Boolean> isFieldOriented){
    swerveDrive = SwerveSubsystem.getInstance();
    this.driveSupp = driveSupp;
    this.strafeSupp = strafeSupp;
    this.turnSupp = turnSupp;
    driveSpeedLimiter = new SlewRateLimiter(myConstants.SwerveSubsystemKs.maxDriveAcc);
    turnSpeedLimiter = new SlewRateLimiter(myConstants.SwerveSubsystemKs.maxTurnAcc);
    strafeSpeedLimiter = new SlewRateLimiter(myConstants.SwerveSubsystemKs.maxStrafeAcc);



    this.isFieldOriented = isFieldOriented;
    
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.initRobot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * @since our velocities are variable, depending on our joystick values triggered inside our robot container, 
   * we may use suppliers to retrieve our data.
   * In our execute method, we:
   *  take our suppliers, 
   * turn them into speed values, 
   * limit them, 
   * convert them to ChassisSpeed objects, 
   * turn our ChassisSpeed objects into a SwerveModuleState array,
   * 'send' this array to our subsystem.
   */
  @Override
  public void execute() {

    // retrieve our velocity suppliers using get() method.
    double driveVel = driveSupp.get();
    double turnVel = turnSupp.get();
    double strafeVel = strafeSupp.get();
  
    // apply a slew rating to our speeds to smooth out speed change.
    driveVel = driveSpeedLimiter.calculate(driveVel);
    strafeVel = driveSpeedLimiter.calculate(strafeVel);
    turnVel = driveSpeedLimiter.calculate(turnVel);

    /**
     * create a new ChassisSpeed objects depending on our odometry reference:
     *  - isFieldOriented or not.
     * Our ChassisSpeeds object takes in our "formated" velocities, our heading 
     * (either as a Rotation2d object or an angle).
    */
    ChassisSpeeds swerveDriveSpeeds = isFieldOriented.get() ? 
    ChassisSpeeds.fromFieldRelativeSpeeds(
      driveVel, 
      strafeVel,
      turnVel,
      swerveDrive.getRotation2d()
    ) : new ChassisSpeeds(driveVel,strafeVel,turnVel); 

    /**
     * Our ChassisSpeeds object is now converted into a SwerveModuleStates (contained in an array as our
     * subsystem requieres them to be).
     */
    SwerveModuleState[] desiredStates = SwerveSubsystemKs.SwerveKinematics.swerveDriveKinematics.toSwerveModuleStates(swerveDriveSpeeds);

    // WE FINALLY set our states to this new array.
    swerveDrive.setModuleStates(desiredStates);

    /* 
    if(isFieldOriented.get()){
      swerveDriveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVel, 
        strafeVel,
        turnVel,
        swerveDrive.getRotation2d()
      );
    } else {
      swerveDriveSpeeds = new ChassisSpeeds(
        driveVel,
        strafeVel,
        turnVel
      );
    }*/

    
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

