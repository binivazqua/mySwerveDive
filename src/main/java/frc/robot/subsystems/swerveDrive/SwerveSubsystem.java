package frc.robot.subsystems.swerveDrive;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.myConstants;;

public class SwerveSubsystem extends SubsystemBase {

    // navX initialization
    AHRS navX = new AHRS(SPI.Port.kMXP);

    // FRONT LEFT MODULE : 1
    private swerveModule frontLeftModule = new swerveModule(
      myConstants.SwerveModuleKs.FrontLeftModuleKs.DriveID, 
      myConstants.SwerveModuleKs.FrontLeftModuleKs.TurnID, 
      1, 
      myConstants.SwerveModuleKs.FrontLeftModuleKs.driveInverted,
      myConstants.SwerveModuleKs.FrontLeftModuleKs.turnInverted
    );

    // FRONT RIGHT MODULE : 2
    private swerveModule frontRightModule = new swerveModule(
      myConstants.SwerveModuleKs.FrontRightModuleKs.DriveID, 
      myConstants.SwerveModuleKs.FrontRightModuleKs.TurnID,
      2, 
      myConstants.SwerveModuleKs.FrontRightModuleKs.driveInverted,
      myConstants.SwerveModuleKs.FrontRightModuleKs.turnInverted
    );

    // BACK LEFT MODULE : 3
    private swerveModule backLeftModule = new swerveModule(
      myConstants.SwerveModuleKs.BackLeftModuleKs.DriveID, 
      myConstants.SwerveModuleKs.BackLeftModuleKs.TurnID, 
        3,
      myConstants.SwerveModuleKs.BackLeftModuleKs.driveInverted,
      myConstants.SwerveModuleKs.BackLeftModuleKs.turnInverted
    );

    // BACK RIGHT MODULE : 4
    private swerveModule backRightModule = new swerveModule(
      myConstants.SwerveModuleKs.BackRightModuleKs.DriveID, 
      myConstants.SwerveModuleKs.BackRightModuleKs.TurnID, 
        4, 
      myConstants.SwerveModuleKs.BackLeftModuleKs.driveInverted, 
      myConstants.SwerveModuleKs.BackRightModuleKs.turnInverted
    );

    // SWERVEDRIVE KINEMATICS OBJECT...
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        myConstants.SwerveSubsystemKs.SwerveKinematics.frontLeftModuleCoords,
        myConstants.SwerveSubsystemKs.SwerveKinematics.frontRightModuleCoords,
        myConstants.SwerveSubsystemKs.SwerveKinematics.backLeftModuleCoords,
        myConstants.SwerveSubsystemKs.SwerveKinematics.backRighttModuleCoords 
    );

    // We create a swervedrive instance 
    private static SwerveSubsystem instance;



  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {
    System.out.println("Swerve Subsystem has been properly initialized");
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double loggingStates[] = {
        0,
        3,
        45,
        3,
        90,
        3,
        120,
        3,
    };

    SmartDashboard.putNumberArray("Swerve States", loggingStates);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /* +++++++++++++++++++++ Swerve inherent methods ++++++++++++ */

  /**
   * Get instance()
   * @return instance of Swerve Subsystem if null (one in use)
   */

  public static SwerveSubsystem getInstance(){
    if(instance == null){
      instance = new SwerveSubsystem();
    }
    return instance;
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

  /* ++++++++++++++++++++ NavX Methods +++++++++++++++++++ */

  /**
   * Get heading()
   * @return my bot's heading
   */
  public double getHeading(){
    return navX.getAngle();
  }

  /**
   * Reset Heading
   * Turns our bot's heading to zero.
   */
  public void resetHeading(){
    navX.reset();
  }

  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++ */

  /* ++++++++++++++++++++ Module inherent methods +++++++++++++++++ */

  /**
   * getModuleState()
   * @return Each module's current state.
   */
  private SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] myModuleStates  = new SwerveModuleState[] {
      frontRightModule.getState(),
      frontLeftModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState(),
    };

    return myModuleStates;
  }

  /**
   * Stop()
   * Stops all modules at once without calling each one of 'em.
   * Prints a statement to debug.
   */

  public void stop(){
    frontRightModule.stopModule();
    frontLeftModule.stopModule();
    backLeftModule.stopModule();
    backRightModule.stopModule();

    System.out.println("All modules stopped.");
  }

  /**
   * Reset Module Heading
   * Resets all modules encoders at once without calling each one of 'em.
   * Prints a statement to debug.
   */
  public void resetModules(){
    frontRightModule.resetEncoders();
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();

    System.out.println("All encoders are reseted.");
  }









  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  


}

