package frc.robot.subsystems.swerveDrive;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.myConstants;
import frc.robot.utilities.myConstants.SwerveSubsystemKs.SwerveKinematics;;

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
      myConstants.SwerveModuleKs.BackRightModuleKs.driveInverted, 
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

    private Joystick controlDriver;

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem(Joystick control) {
    System.out.println("Swerve Subsystem has been properly initialized");
    this.controlDriver = control;

    
  }

  

  @Override
  public void periodic() {

    ChassisSpeeds newDesiredChassisSpeeds = new ChassisSpeeds(
      controlDriver.getRawAxis(1),
      controlDriver.getRawAxis(0),
      controlDriver.getRawAxis(2)
    );

    setChassisSpeeds(newDesiredChassisSpeeds);

   

    // This method will be called once per scheduler run
    double loggingStates[] = {
        frontLeftModule.getState().angle.getDegrees(),
        frontLeftModule.getState().speedMetersPerSecond,
        frontRightModule.getState().angle.getDegrees(),
        frontRightModule.getState().speedMetersPerSecond,
        backLeftModule.getState().angle.getDegrees(),
        backLeftModule.getState().speedMetersPerSecond,
        backRightModule.getState().angle.getDegrees(),
        backRightModule.getState().speedMetersPerSecond,
        
    };

     // This method will be called once per scheduler run
     double testStates[] = {
      45,
      1,
      45,
      1,
      45,
      1,
      45,
      5
     };
      
  


    SwerveModuleState states[] = {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };

      SmartDashboard.putNumberArray("MODULE STATES", loggingStates);
    //System.out.println("Swerve States: "+ loggingStates);
    System.out.println("MODLE 1 ROT: "+ frontLeftModule.getState().angle.getDegrees());
    //System.out.println("MODULE 1 DRIVE: " + frontLeftModule.getState().speedMetersPerSecond);


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

    /* 
  public static SwerveSubsystem getInstance(){
    if(instance == null){
      instance = new SwerveSubsystem();
    }
    return instance;
  }
  */

  public void setChassisSpeeds(ChassisSpeeds myChassisSpeeds){
    SwerveModuleState[] newStates = SwerveKinematics.swerveDriveKinematics.toSwerveModuleStates(myChassisSpeeds);

    //Set Chassis Speeds
    frontLeftModule.setDesiredState(newStates[0]);
    frontRightModule.setDesiredState(newStates[1]);
    backLeftModule.setDesiredState(newStates[2]);
    backRightModule.setDesiredState(newStates[3]);

    
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

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
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

  public void setModuleStates(SwerveModuleState[] desiredStates){

    myConstants.SwerveSubsystemKs.SwerveKinematics.swerveDriveKinematics.desaturateWheelSpeeds(desiredStates, myConstants.SwerveSubsystemKs.maxDriveSpeed);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /* ++++++++++++++++++++++++ command funcs ++++++++++++++++++++++ */

  public void initRobot(){
    resetHeading();
    resetModules();
    System.out.println("Modules and heading reseted.");
  }









  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  


}

