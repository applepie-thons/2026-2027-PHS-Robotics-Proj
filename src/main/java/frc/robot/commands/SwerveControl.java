package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Meant to be a background command that runs when no other commands are run.
 * 
 * Lets the robot be controlled by a controller; basically Teleop mode.
 */
public class SwerveControl extends Command {
    

    public SwerveControl(CommandXboxController m_driveController){
        
    }


    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean Interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
