package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.climber.climber;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.subsystems.vision.CustomGamePieceVision;
import frc.robot.subsystems.intake.Intake;

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
     public Intake m_intake = new Intake();
     public climber m_climber = new climber();
     //public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_pipeline");
    
    /* Commands */
    public CommandIntakeIn commandIntakeIn = new CommandIntakeIn(m_intake);
    public CommandIntakeOut commandIntakeOut = new CommandIntakeOut(m_intake);
    public CommandIntakeStop commandIntakeStop = new CommandIntakeStop(m_intake);
    public CommandSwerveGoToHeading commandSwerveHeading0 = new CommandSwerveGoToHeading(0, s_Swerve);
    public CommandSwerveGoToHeading commandSwerveHeading90 = new CommandSwerveGoToHeading(90, s_Swerve);
    public CommandSwerveGoToHeading commandSwerveHeading180 = new CommandSwerveGoToHeading(180, s_Swerve);
    public CommandSwerveGoToHeading commandSwerveHeading270 = new CommandSwerveGoToHeading(270, s_Swerve);
    
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
        NamedCommands.registerCommand("marker1", Commands.print("Finished 2 Piece"));
        NamedCommands.registerCommand("marker2", Commands.print("Finished 3-4 Piece"));

        NamedCommands.registerCommand("Start Intake", commandIntakeIn);
        NamedCommands.registerCommand("Stop Intake", commandIntakeStop);
        NamedCommands.registerCommand("Reverse Intake", commandIntakeOut);

        NamedCommands.registerCommand("Go To Podium Positon", Commands.runOnce(() -> m_SAT.goToBasePodiumPosition(), m_SAT));
        NamedCommands.registerCommand("Go To AMP Positon", Commands.runOnce(() -> m_SAT.goToBaseAmpPosition(), m_SAT));
        NamedCommands.registerCommand("Go To Sub Positon", Commands.runOnce(() -> m_SAT.goBaseToSubPosition(), m_SAT));
        NamedCommands.registerCommand("Go To Trap Positon", Commands.runOnce(() -> m_SAT.goToBaseTrapPosition(), m_SAT));
        NamedCommands.registerCommand("Go To Zero Positon", Commands.runOnce(() -> m_SAT.goToBaseZeroPosition(), m_SAT));
         
         
        // NamedCommands.registerCommand("Go To Podium Positon", Commands.runOnce(() -> m_SAT.goToPodiumPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To AMP Positon", Commands.runOnce(() -> m_SAT.goToAmpPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Sub Positon", Commands.runOnce(() -> m_SAT.goToSubPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Trap Positon", Commands.runOnce(() -> m_SAT.goToTrapPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Zero Positon", Commands.runOnce(() -> m_SAT.goToZeroPosition(), m_SAT));

        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    // /**
    //  * Use this method to define your button->command mappings. Buttons can be created by
    //  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    //  * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
    //  * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    //  */
     private void configureButtonBindings() {
         /* Driver Buttons */
         //driver.y()
         //   .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(), s_Swerve));  /// suggest commenting this out while we troubleshoot this
        
         /* Operator Buttons */
         operator.a()
            .onTrue(Commands.runOnce(() -> m_SAT.goToBaseZeroPosition(), m_SAT));

         operator.b()
             .and(operator.axisGreaterThan(1, 0.6))
             .and(operator.axisLessThan(0, 0.4))
            .and(operator.axisGreaterThan(0, -0.4))
          .onTrue(Commands.runOnce(() -> m_SAT.goToBasePodiumPosition(), m_SAT));

        operator.b()
            .and(operator.axisLessThan(1, -0.6))
            .and(operator.axisLessThan(0, 0.4))
            .and(operator.axisGreaterThan(0, -0.4))
            .onTrue(Commands.runOnce(() -> m_SAT.goBaseToSubPosition(), m_SAT));
        
        operator.b()
            .and(operator.axisLessThan(0, -0.6))
            .and(operator.axisLessThan(1, 0.4))
            .and(operator.axisGreaterThan(1, -0.4))
            .onTrue(Commands.runOnce(() -> m_SAT.goToBaseTrapPosition(), m_SAT));

        operator.b()
            .and(operator.axisGreaterThan(0, 0.6))
            .and(operator.axisLessThan(1, 0.4))
            .and(operator.axisGreaterThan(1, -0.4))
            .onTrue(Commands.runOnce(() -> m_SAT.goToBaseAmpPosition(), m_SAT));

         operator.start()
             .onTrue(commandIntakeIn);

         operator.back()
             .onTrue(commandIntakeOut);
        
        operator.start()
            .and(operator.back())
            .onTrue(commandIntakeStop)
            .onFalse(commandIntakeStop);

        driver.leftTrigger()
            .onTrue(Commands.runOnce(() -> m_SAT.shootNote(), m_SAT));
         
        driver.povUp()
            .onTrue(commandSwerveHeading0);
        driver.povLeft()
            .onTrue(commandSwerveHeading90);
        driver.povDown()
            .onTrue(commandSwerveHeading180);
        driver.povRight()
            .onTrue(commandSwerveHeading270);
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