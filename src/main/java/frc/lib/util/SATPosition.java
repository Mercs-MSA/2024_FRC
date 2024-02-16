package frc.lib.util;

import frc.robot.Constants.SATConstants;

public class SATPosition {
    public final double baseMotor1TargetPos;
    public final double baseMotor2TargetPos;
    public final double pivotTargetPos;

    /**
     * SAT Position structure.
     * @param baseMotor1TargetPos
     * @param baseMotor2TargetPos
     * @param pivotTargetPos
     */
    public SATPosition(SATConstants.Position position) throws Exception {
        switch (position) {
            case PODIUM:
                this.baseMotor1TargetPos = SATConstants.MOTOR1_BASE_PODIUM_POS;
                this.baseMotor2TargetPos = SATConstants.MOTOR2_BASE_PODIUM_POS;
                this.pivotTargetPos = SATConstants.PIVOT_PODIUM_POS;
                break;
            case SUB:
                this.baseMotor1TargetPos = SATConstants.MOTOR1_BASE_SUB_POS;
                this.baseMotor2TargetPos = SATConstants.MOTOR2_BASE_SUB_POS;
                this.pivotTargetPos = SATConstants.PIVOT_SUB_POS;
                break;
            case AMP:
                this.baseMotor1TargetPos = SATConstants.MOTOR1_BASE_AMP_POS;
                this.baseMotor2TargetPos = SATConstants.MOTOR2_BASE_AMP_POS;
                this.pivotTargetPos = SATConstants.PIVOT_AMP_POS;
                break;
            case TRAP:
                this.baseMotor1TargetPos = SATConstants.MOTOR1_BASE_TRAP_POS;
                this.baseMotor2TargetPos = SATConstants.MOTOR2_BASE_TRAP_POS;
                this.pivotTargetPos = SATConstants.PIVOT_TRAP_POS;
                break;
            case START:
                this.baseMotor1TargetPos = SATConstants.MOTOR1_BASE_START_POS;
                this.baseMotor2TargetPos = SATConstants.MOTOR2_BASE_START_POS;
                this.pivotTargetPos = SATConstants.PIVOT_START_POS;
                break;
            default:
                throw new Exception("Invalid Position");
        }
    }
}