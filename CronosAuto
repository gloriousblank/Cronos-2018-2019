import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Time;
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

@Autonomous (name="BasicAuto", group="Auto Opmode")
//@Disabled
public class CronosAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;
    private DcMotor  climb= null;
    private DcMotor arm = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRight  = hardwareMap.get(DcMotor.class, "fr");
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        rearRight  = hardwareMap.get(DcMotor.class, "rr");
        rearLeft  = hardwareMap.get(DcMotor.class, "rl");
        arm  = hardwareMap.get(DcMotor.class, "arm");
        climb  = hardwareMap.get(DcMotor.class, "climb");


        // Most robots need both motors on one side to be reversed and the others to be set to forward to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // sets the time for the run time of the wheels to get to the claim zone
            //Phase 1 Automouos
            while (runtime.time(TimeUnit.SECONDS) <= 2.0) {
                // Send calculated power of .5 to all wheels
                // Move forward
                if (runtime.time(TimeUnit.SECONDS) < 1.5) {
                    setWheelPower(-0.75);
                }
                else {
                    setWheelPower(0.0);
                }
            }
            //Sends Data to the phones for debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString(), runtime.time());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }



    }
    // Methods
    private void setWheelPower(double v) {
        frontLeft.setPower(v);
        frontRight.setPower(v);
        rearLeft.setPower(v);
        rearRight.setPower(v);
    }

}


