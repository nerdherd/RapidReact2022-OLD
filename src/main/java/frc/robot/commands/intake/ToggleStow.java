package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleStow extends CommandBase {
    private IntakeBalls intake;
    private Stow stow;
    private boolean isStow = false; // true while intaking

    public ToggleStow() {
        intake = new IntakeBalls();
        stow = new Stow();
        
    }

    @Override
    public void initialize() {
        stow.initialize();
        intake.initialize();
    }

    @Override
    public void execute() {
        if (isStow) {
            stow.execute();
        } else {
            intake.execute();
        }
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
