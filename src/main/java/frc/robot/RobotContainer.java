package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.vision.CustomGamePieceVision;
import frc.robot.subsystems.intake.intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public SAT m_SAT = new SAT();
    public intake m_Intake = new intake();
    public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_pipeline");

     /* AutoChooser */
     private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(), 
                () -> -driver.getRightX(), 
                () -> driver.leftBumper().getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
        
        //Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
         
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y()
            .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(), s_Swerve));  /// suggest commenting this out while we troubleshoot this
        
        /* Operator Buttons */
        operator.a()
            .whileTrue(m_SAT.goToZeroPosition());

        operator.leftBumper()
            .and(operator.axisGreaterThan(1, 0.6))
            .and(operator.axisLessThan(0, 0.4))
            .and(operator.axisGreaterThan(0, -0.4))
            .onTrue(m_SAT.goToPodiumPosition());

        operator.leftBumper()
            .and(operator.axisLessThan(1, -0.6))
            .and(operator.axisLessThan(0, 0.4))
            .and(operator.axisGreaterThan(0, -0.4))
            .onTrue(m_SAT.goToSubPosition());

        operator.leftBumper()
            .and(operator.axisLessThan(0, -0.6))
            .and(operator.axisLessThan(1, 0.4))
            .and(operator.axisGreaterThan(1, -0.4))
            .onTrue(m_SAT.goToAmpPosition());

        operator.leftBumper()
            .and(operator.axisGreaterThan(0, 0.6))
            .and(operator.axisLessThan(1, 0.4))
            .and(operator.axisGreaterThan(1, -0.4))
            .onTrue(m_SAT.goToTrapPosition());

        operator.b()
            .onTrue(new IntakeMoveCommand(m_Intake.positionArmDown))
            .onFalse(new IntakeMoveCommand(m_Intake.positionArmUp));

        operator.x()
            .onTrue(m_Intake.rollerSpeed(m_Intake.speedRollerInward));

        operator.y()
            .onTrue(m_Intake.rollerSpeed(m_Intake.speedRollerOutward));
        
        operator.y()
            .and(operator.x())
            .onTrue(m_Intake.rollerSpeed(m_Intake.speedRollerOff))
            .onFalse(m_Intake.rollerSpeed(m_Intake.speedRollerOff));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
