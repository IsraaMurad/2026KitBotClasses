
package frc.robot.commands;
//import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Command to drive a tank-style (differential) robot straight for a specified distance
 * in meters, using ONLY NAVX2 (AHRS) sensor input.
 *
 * - Heading correction uses NAVX2 yaw (getYaw()) with a PID controller to keep the robot straight.
 * - Distance is measured using NAVX2's experimental displacement estimates (getDisplacementX/Y).
 *   The magnitude of the horizontal displacement vector is used as the "driven distance".
 *   NOTE: Displacement is experimental and subject to integration drift/noise (~0.1m error over ~15s).
 *         This meets the "use only NAVX2 input" requirement but is not as accurate as wheel encoders.
 *
 * Usage:
 *   - Add the navX vendor library to your project (via VS Code / Gradle).
 *   - Instantiate with your DifferentialDrive object and desired distance.
 *   - Schedule the command (e.g., in autonomous).
 */
public class DriveStraight extends Command {

    private final CANDriveSubsystem m_drivetrain;
    
    private final double m_targetDistanceMeters;

    private final PIDController m_headingPID;

    private final double driveSpeed;

    //private final AHRS m_navx;
    // Tunable constants (adjust based on your robot)
    //private static final double FORWARD_SPEED = 0.5;   // Base forward speed (0.0 to 1.0)
    private static final double kP = 0.015;            // Proportional gain for heading correction
    private static final double kI = 0.0;              // Integral (usually 0 for short straight drives)
    private static final double kD = 0.001;            // Derivative

    
    /**
     * @param drivetrain Your robot's DifferentialDrive instance (tank drive)
     * @param driveDistanceMeters Distance to travel straight (positive = forward)
     */
    public DriveStraight(CANDriveSubsystem drivetrain, double driveDistanceMeters, double dS) {
        drivetrain.driveStraight(dS);
        driveSpeed = dS;
        m_drivetrain = drivetrain;
        m_targetDistanceMeters = Math.abs(driveDistanceMeters); // Ensure positive

        // NAVX2 on the MXP port (most common for navX2-MXP; use I2C.Port.kMXP if wired that way)
        //m_navx = drivetrain.m_navx;
        
    

        m_headingPID = new PIDController(kP, kI, kD);
        m_headingPID.setTolerance(2.0); // Degrees tolerance for "on target"

        // Since yaw wraps -180..180, enable continuous input for smooth PID
        m_headingPID.enableContinuousInput(-180.0, 180.0);

        // This command requires the drivetrain (in a real subsystem you'd addRequirements)
        // For standalone example we assume the caller handles it.
    }


    @Override
    public void initialize() {
        m_drivetrain.initializeNavX();
        
        
    }

    @Override
    public void execute(){
        m_drivetrain.driveStraight(driveSpeed);
    }

    @Override
    public boolean isFinished(){
        return m_drivetrain.checkFinished(m_targetDistanceMeters);
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("This command was interrupted: "+interrupted);
        m_drivetrain.tankDrive(0, 0);
        m_headingPID.close();
    }
}


