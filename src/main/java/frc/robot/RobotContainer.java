package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandOverrideIndexStart;
import frc.robot.commands.CommandOverrideIndexStop;
import frc.robot.commands.CommandOverrideIntakeStart;
import frc.robot.commands.CommandOverrideIntakeStop;
import frc.robot.commands.CommandShootNote;
import frc.robot.commands.CommandStopShooter;
import frc.robot.commands.CommandPivotPosition;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.IndexSubcommands.CommandIndexIdle;
import frc.robot.commands.IndexSubcommands.CommandIndexProcess;
import frc.robot.commands.IndexSubcommands.CommandIndexStart;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.climber.climber;
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
    public static final Swerve s_Swerve = new Swerve();
    public final SAT m_SAT = new SAT();
    public final Intake m_intake = new Intake();
    public final climber m_climber = new climber();
    //public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_pipeline");

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            put("Start Intake", new CommandOverrideIntakeStart(m_intake));
            put("Start Index", new CommandOverrideIndexStart(m_intake));
            put("Stop Intake", new CommandOverrideIntakeStop(m_intake));
            put("Stop Index", new CommandOverrideIndexStop(m_intake));
            put("Start Shooter", new CommandShootNote(m_SAT));
            put("Stop Shooter", new CommandStopShooter(m_SAT));
        }  
    };

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

        // Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Finished 1 Piece"));
        NamedCommands.registerCommand("marker2", Commands.print("Finished 3-4 Piece"));

        NamedCommands.registerCommands(autonomousCommands);

        NamedCommands.registerCommand("Intake Note", new SequentialCommandGroup(
            new CommandPivotPosition("handoff", m_SAT),
            m_intake.collectNote(),
            m_intake.passNoteToIndex(),
            new CommandPivotPosition("start", m_SAT)
            ));
         
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void configureButtonBindings() {
        driverControls();
        operatorControls();
    }

    public void driverControls(){
        driver.start().and(driver.back()).onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));

 

    }

    public void operatorControls(){
        // operator.pov(90).whileTrue(new RunCommand(() -> m_SAT.pivotGoToPositionIncrement(0.25), m_SAT));
        // operator.pov(270).whileTrue(new RunCommand(() -> m_SAT.pivotGoToPositionIncrement(-0.25), m_SAT));
        // operator.pov(90).whileTrue(new RunCommand(() -> m_SAT.baseGoToPositionIncrement(0.5), m_SAT));
        // operator.pov(270).whileTrue(new RunCommand(() -> m_SAT.baseGoToPositionIncrement(-0.5), m_SAT));

        // operator.pov(0).onTrue(new CommandPivotPosition("podium", m_SAT));
        // operator.pov(90).onTrue(new CommandPivotPosition("start", m_SAT));
        // operator.pov(270).onTrue(new CommandPivotPosition("wing", m_SAT));
        // operator.a().onTrue(new CommandPivotPosition("handoff", m_SAT));
        // operator.pov(180).onTrue(new CommandPivotPosition("start", m_SAT));

        // operator.pov(0).onTrue(new InstantCommand(() -> Constants.SATConstants.setState("wing")));
        // operator.pov(90).onTrue(new InstantCommand(() -> Constants.SATConstants.setState("podium")));
        // operator.pov(180).onTrue(new InstantCommand(() -> Constants.SATConstants.setState("sub")));
        // operator.pov(270).onTrue(new InstantCommand(() -> Constants.SATConstants.setState("amp")));

        operator.x()
           .onTrue(
               new SequentialCommandGroup(
                    new CommandPivotPosition("handoff", m_SAT),
                    new CommandIndexStart(m_intake),
                    m_intake.collectNote(),
                    m_intake.passNoteToIndex(),
                    new CommandPivotPosition("start", m_SAT)
                )
        );


        operator.y()
        .onTrue(
            new SequentialCommandGroup(
                new CommandIndexProcess(m_intake),
                new CommandShootNote(m_SAT),
                new CommandIndexStart(m_intake),
                new WaitCommand(.3),
                new CommandIndexIdle(m_intake),
                new CommandStopShooter(m_SAT)
        


            ));

        
     
        // operator.rightBumper()
        //     .onTrue(
        //         new CommandPivotPosition("trap", m_SAT)
        //     );

        // operator.leftBumper()
        //     .onTrue(
        //         new CommandPivotPosition("start", m_SAT)
        //     );

        // operator.y()
        //     .whileTrue(
        //         m_climber.climbDownRightCommand()
        //     );
    
        // operator.x()
        //     .whileTrue(
        //         m_climber.climbUpRightCommand()
        //     );
            
        // operator.a()
        //     .whileTrue(
        //         m_climber.climbUpLeftCommand()
        //     );
    
        // operator.b()
        //     .whileTrue(
        //         m_climber.climbDownLeftCommand()
        //     );
    }

    public void operatorTesting(){
        /************************/
        /*                      */
        /*   Operator Buttons   */
        /*                      */
        /************************/

/*         operator.axisGreaterThan(5, 0.5)
            .whileTrue(
                m_climber.climbDownCommand()
            );
 */
        // operator.axisLessThan(5, -0.5)
        //     .whileTrue(
        //         m_climber.climbUpCommand()
        //     );

        // operator.axisLessThan(5, 0.5)
        //     .whileTrue(
        //         m_climber.climbMotorStop()
        //     );


        
        // if (m_intake.simulationDebugMode) {
        //     operator.a()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setLowerSensorDetectsNote(true);}));
        //     operator.b()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setUpperSensorDetectsNote(true);}));
        //     operator.x()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setLowerSensorDetectsNote(false);}));
        //     operator.y()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setUpperSensorDetectsNote(false);}));
        // }
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