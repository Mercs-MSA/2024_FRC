package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climber.climber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climbUpCommand = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton climbDownCommand = new JoystickButton(driver, XboxController.Button.kBack.value);
    
    /* Subsystems */
    public Swerve s_Swerve = new Swerve();
    public climber m_Climber = new climber();

    /* Auto Chooser */
     private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                            new PIDConstants(5.0, 0.0, 0.0),
                                            // Translation PID constants
                                            new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                                            swerveDrive.swerveController.config.headingPIDF.i,
                                                            swerveDrive.swerveController.config.headingPIDF.d),
                                            // Rotation PID constants
                                            4.5,
                                            // Max module speed, in m/s
                                            // Drive base radius in meters. Distance from robot center to furthest module.
                                            0.3787, //meters
                                            // DAN CHANGED THE ABOVE BECAUSE THIS METHOD DIDN'T EXIST; OLD VERSION IS IN COMMENT BELOW
                                            //swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                            new ReplanningConfig()
                                            // Default path replanning config. See the API for the options here
        ),
        null,
        this // Reference to this subsystem to set requirements
        );

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

         // Register Named Commands
        // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));  /// suggest commenting this out while we troubleshoot this
        climbUpCommand.onTrue(new InstantCommand(() -> m_Climber.climbUpCommand()));
        climbDownCommand.onTrue(new InstantCommand(() -> m_Climber.climbDownCommand()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    //     // An ExampleCommand will run in autonomous
    //     return new PathPlannerAuto("Example Auto");
    // }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPathWithEvents(PathPlannerPath)} path command.
     */
    public void getAutonomousCommand(String pathName, boolean setOdomToStart) {
        s_Swerve.getAutonomousCommand(pathName, setOdomToStart);
    }
}
