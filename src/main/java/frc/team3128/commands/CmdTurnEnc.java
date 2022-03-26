package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdTurnEnc extends CommandBase {

    private NAR_Drivetrain drivetrain;
    private double turnMeters;

    private double initLeftPosMeters, initRightPosMeters;

    private double turnDegrees;

    public CmdTurnEnc(NAR_Drivetrain drivetrain, double turnDegrees) {

        this.drivetrain = drivetrain;
        this.turnDegrees = turnDegrees;

        initLeftPosMeters = drivetrain.getLeftEncoderDistance(); //meters
        initRightPosMeters = drivetrain.getRightEncoderDistance();

        addRequirements(drivetrain);

        turnMeters = this.turnDegrees * 0.92155 / 180;
    }

    @Override
    public void execute() {
        if (turnMeters > 0.1) {
            drivetrain.tankDrive(-0.15, 0.15);
        } 
        else {
            drivetrain.tankDrive(0, 0);
        }

        
    }

    public boolean isFinished() {
        //return Math.abs((drivetrain.getLeftEncoderDistance() - initLeftPosMeters) - (drivetrain.getRightEncoderDistance() - initRightPosMeters)) / 2 < 0.1;
        //return (drivetrain.getLeftEncoderDistance() - initLeftPosMeters)



        if (drivetrain.getLeftEncoderDistance() < -turnMeters || drivetrain.getRightEncoderDistance() > turnMeters) {
            return true;
        }
        else return false;
    }

}
