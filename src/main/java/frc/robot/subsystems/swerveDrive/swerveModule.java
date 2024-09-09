package frc.robot.subsystems.swerveDrive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class swerveModule {

    private final CANSparkMax turnMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder absEncoder;

    private final PIDController turnPIDController;
    private final int moduleNumber;

 

  /** Creates a new ExampleSubsystem. */
  public swerveModule(int driveMotorID, int turnMotorID, int moduleNum, boolean driveInverted, boolean turnInverted) {

    // Assign module number for debugging purposes:
    this.moduleNumber = moduleNum;

    turnMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

    this.driveEncoder = driveMotor.getEncoder();
    this.turnEncoder = turnMotor.getEncoder();
    this.absEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    this.turnPIDController = new PIDController(0.266, 0, 0);

    // Keep inverted values in a separate  ks file to get easier access to 'em:
    driveMotor.setInverted(driveInverted);
    turnMotor.setInverted(turnInverted);

    // Optimize turning control enabling Continuous Input in radians:
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);


    
  }

  /**
   * Sets current module's speed.
   * @param speed in meters per second.
   */
  public void setSpeed(double speed){
    driveMotor.set(speed);
  }

  
  /**
   *  Sets turn angle to a specific position using PIDController with relative encoder.
   * @param angle 
   */
  public void setAngle(double angle){
    turnMotor.set(turnPIDController.calculate(turnEncoder.getPosition(), angle));
  }

  /**
   * Resets current module's drive encoder postiion to zero.
   * Sets the turn encoder to our Abs encoder position to 
   * never lose its location.
   */
  public void resetEncoders(){
    driveEncoder.setPosition(0);
    // Set turn encoder to abs Encoder to initialize (reset) it:
    turnEncoder.setPosition(absEncoder.getPosition());
  }

  // Create all methods for debugging purposes:

  /**
   * Get Drive Speed()
   * @returns current module's linear speed in m/s
   */
  public double getDriveSpeed(){
    return driveMotor.get();
  }

  /**
   * Get Turn Speed()
   * @returns our current module's angular speed.
   */
  public double getTurnSpeed(){
    return turnMotor.get();
  }
  
  /**
   * Get Absolute Position()
   * @returns our modul'es current absolute encoder position.
   */
  public double getAbsPosition(){
    return absEncoder.getPosition();
  }

  /**
   * Get Drive Position()
   * @returns our current module's drive relative encoder position.
   */
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  /**
   * Get Turn Position()
   * @returns our current module's turn motor relative encoder position.
   */
  public double getTurnPosition(){
    return turnEncoder.getPosition();
  }

  /**
   * Get Drive Inverted()
   * @returns our current module's drive motor state: inverted or not.
   */
  public boolean getDriveInverted(){
    return driveMotor.getInverted();
  }

  /**
   * Get Turn Inverted()
   * @returns our current module's turn motor state: inverted or not.
   */
  public boolean getTurnInverted(){
    return turnMotor.getInverted();
  }

  /**
   * Stop ()
   * Sets our module's velocity to zero.
   */
  public void stopModule(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  /**
   * Set Desired State()
   * @param desiredState is a SwerveModule object created in our Swerve Subsystem,
   * which uses our kinematics to set the accurate state desired for our module to
   * be in.
   * Sets a controller deadband to override all values less than 0.05.
   */
  public void setDesiredState(SwerveModuleState desiredState){

    // Joystick deadband to override all values lees than 0.05 (save in a ks file)
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.05){
        stopModule();
    }

    // get desired state's properties and set them to motors.
    driveMotor.set(desiredState.speedMetersPerSecond / 6); // Go to full velocity. s
    setAngle(desiredState.angle.getRadians());

  }

  /**
   * Get Module State
   * @returns the requestes module's current state, composed by:
   * Drive velocity and angle.
   */
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurnPosition()));
  }

  
}

