import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DepotHang", group="Auto Opmode")
//@Disabled
//Special thanks to Dr. Santamaria (mentor). Helped create autonomous and very useful methods.
public class DepotCaseAuto extends LinearEncoder {
public enum BehaviorState {
    Lower, //Lower from Hanger
    Unhook, //Unhook from Hanger
    Orient, // Return back 0 heading
    DriveToDepot, //Drive to Depot
    Turn,//Turn
    Claim, //Raise Arms to claim object
    TurnBack,
    DriveBack, //Backup
    TurnToCrater, //turn Robot to crater
    MoveToCrater, //go to crater
    Stop //Stop
}
public BehaviorState state;

    public void runOpMode() {
        state = BehaviorState.DriveToDepot;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        initializeHardware();
        resetAngle();
       //moveArmToPos(-250,0.5);

    // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch(state) {
                case Lower: {
                    moveElevatorToPos(0);
                    if(climb.getCurrentPosition() < 5) {
                        state = BehaviorState.Unhook;
                    }
                }
                break;

                case Unhook: {
                    // define goals
                    //35 degrees (positive is left, negative is right)
                    double target = 35.0;
                    double turn = 0.5 * Range.clip(-(target - getAngle()) / 25.0, -1, 1);

                    double sign = Math.signum(turn);
                    turn = sign * Math.max(Math.abs(turn), 0.05);
                    setDrivePower(0.0, 0.0, turn);

                    if (  Math.abs(target - getAngle()) < 3.0) {
                        // next state
                        state = BehaviorState.Orient;
                    }
                }
                break;

                case Orient:{
                    // define goals
                    //Turn to 0 degrees (positive is left, negative is right)
                    double target = 0.0;
                    double turn = 0.5 * Range.clip(-(target - getAngle()) / 25.0, -1, 1);

                    double sign = Math.signum(turn);
                    turn = sign * Math.max(Math.abs(turn), 0.05);
                    setDrivePower(0.0, 0.0, turn);

                    if (  Math.abs(target - getAngle()) < 3.0) {
                        // next state
                        resetAngle();
                        state = BehaviorState.DriveToDepot;
                    }
                }
                break;

                case DriveToDepot: {
                    //move forward 20 inches into the depot
                    float distance = 35f;
                    int   target   = (int)(distance / (Math.PI * 6) * 280.0 * 0.95);
                    moveRobotForwardToPos(distance, 0.5);
                    int pos = (rearLeft.getCurrentPosition() + rearRight.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 4;
                    if (pos > target ) {
                        // next state
                        setDrivePower(0.0, 0.0, 0.0);
                        resetDriveEncoders();
                        state = BehaviorState.Claim;

                    }
                }
                break;

                case Turn: {
                    double targetA = 180.0;
                    double turnA = 0.5 * Range.clip(-(targetA - getAngle()) / 25.0, -1, 1);

                    double signA = Math.signum(turnA);
                    turnA = signA * Math.max(Math.abs(turnA), 0.05);
                    setDrivePower(0.0, 0.0, turnA);

                    if (  Math.abs(targetA - getAngle()) < 5.0) {
                        // next state
                        resetAngle();
                        setDrivePower(0.0,0.0,0.0);
                        delay(1000);
                        state = BehaviorState.Claim;
                    }
                }
                break;

                case Claim: {
                    delay(1000);
                    moveArmToPos(-600, 0.5
                    );
                    delay(1000);
                    moveArmToPos(-250, 0.59);
                    delay(1000);
                    state = BehaviorState.DriveBack;
                }

                break;

                case TurnBack: {


                    double targetB = 180.0;
                    double turnB = 0.5 * Range.clip(-(targetB - getAngle()) / 25.0, -1, 1);

                    double signB = Math.signum(turnB);
                    turnB = signB * Math.max(Math.abs(turnB), 0.05);
                    setDrivePower(0.0, 0.0, turnB);

                    if (  Math.abs(targetB - getAngle()) < 5.0) {
                        // next state
                        turnB = 0;
                        resetAngle();
                        setDrivePower(0.0,0.0,0.0);
                        moveArmToPos(-200,.59);
                        state = BehaviorState.DriveBack;
                    }
                }

                break;

                case DriveBack: {
                    float distance = -15f;
                    int   target   = (int)(distance / (Math.PI * 6) * 280.0 * 0.95);
                    moveRobotForwardToPos(distance , 0.5);
                    int pos = (rearLeft.getCurrentPosition() + rearRight.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 4;
                    if (Math.abs(pos) > Math.abs(target)) {
                        // next state
                        resetAngle();
                        setDrivePower(0.0, 0.0, 0.0);
                        resetDriveEncoders();
                        delay(1000);
                        state = BehaviorState.Stop;

                    }
                }
                break;
                case TurnToCrater: {
                    // define goals
                    //40 degrees (positive is left, negative is right)
                    double targetC = 40.0;
                    double turnC = 0.5 * Range.clip(-(targetC - getAngle()) / 25.0, -1, 1);

                    double sign = Math.signum(turnC);
                    turnC = sign * Math.max(Math.abs(turnC), 0.05);
                    setDrivePower(0.0, 0.0, turnC);

                    if (  Math.abs(targetC - getAngle()) < 5.0) {
                        // next state
                        resetAngle();
                        delay(1000);
                        state = BehaviorState.MoveToCrater;
                    }

                }
                break;
                case MoveToCrater: {
                    float distance =120f;
                    int   target   = (int)(distance / (Math.PI * 6) * 280.0 * 0.95);
                    //Move to Crater
                    moveRobotForwardToPos(distance , 0.75);
                    int pos = (rearLeft.getCurrentPosition() + rearRight.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 4;
                    if (Math.abs(pos) > Math.abs(target)) {
                        // next state
                        setDrivePower(0.0, 0.0, 0.0);
                        resetDriveEncoders();
                        delay(1000);
                        state = BehaviorState.Stop;

                    }
                }
                break;
                case Stop: {
                    //Stops the Robot in the crater
                    setDrivePower(0.0, 0.0, 0.0);
                }
                break;
            }
        }



    }

}


