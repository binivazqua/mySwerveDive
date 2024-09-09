package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class myConstants {
    
    public static final class SwerveModuleKs {

        public static final class ConversionFactorKs {
            // our swerve drive wheels are 4 inches width.
            public static final double wheelDiameterMeters = Units.inchesToMeters(4); 
            /**
             * 1 : 73 ratio, which means that every 7.13 motor rotations, the wheel makes 1 full rot.
             */
            public static final double driveMotorGearRatio = 1 / 7.13; 

            /**
             * 1 : 13.71 ratio, which means that every 13.71 motor rotations, the TURN wheel makes 1 full rot.
             */
            public static final double m = 1 / 13.71; 

        }

        /* +++++++++++++++++++++++ MODULE INDIVIDUAL Ks +++++++++++++++++++++ */
        // front right module
        public static final class FrontRightModuleKs {
            public static final int DriveID = 1;
            public static final int TurnID = 2;

            public static final boolean driveInverted = false;
            public static final boolean turnInverted = false;

        }
        // front left module
        public static final class FrontLeftModuleKs {
            public static final int DriveID = 3;
            public static final int TurnID = 4;

            public static final boolean driveInverted = false;
            public static final boolean turnInverted = false;
        }
        
        // back right module
        public static final class BackRightModuleKs {
            public static final int DriveID = 5;
            public static final int TurnID = 6;

            public static final boolean driveInverted = false;
            public static final boolean turnInverted = false;
        }
        
        // back left module
        public static final class BackLeftModuleKs {
            public static final int DriveID = 7;
            public static final int TurnID = 8;

            public static final boolean driveInverted = false;
            public static final boolean turnInverted = false;
        }
            
    }

    public static final class SwerveSubsystemKs { 

        public static final double maxDriveSpeed = 6;
        public static final double maxDriveAcc = 6;

        public static final double maxStrafeAcc = 6;
        public static final double maxTurnAcc = 6;

        public static final double kTurnDrive = 0.255;



        // separate kinematics class to organize our SwerveDrive subsystem
        public static class SwerveKinematics {
            
            // chassis dimensions in inches
            public static final double trackWidth = 20;
            public static final double wheelBase = 20;

            // Swerve Drive Kinematics Coordinates 
            
            
            public static final Translation2d frontRightModuleCoords= new Translation2d(wheelBase / 2, trackWidth / 2);
            public static final Translation2d frontLeftModuleCoords= new Translation2d(wheelBase / 2, - trackWidth / 2);
            public static final Translation2d backLeftModuleCoords= new Translation2d(- wheelBase / 2, trackWidth / 2);
            public static final Translation2d backRighttModuleCoords= new Translation2d(- wheelBase / 2, - trackWidth / 2);
        

            public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2),
                new Translation2d(wheelBase / 2, - trackWidth / 2),
                new Translation2d(- wheelBase / 2, trackWidth / 2),
                new Translation2d(- wheelBase / 2, - trackWidth / 2));
        }
        
    }

   

    
}

