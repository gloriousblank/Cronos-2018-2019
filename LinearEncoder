/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@TeleOp(name="TeleOP", group="Linear Opmode")
//@Disabled
//Special thanks to Dr. Santamaria (mentor). Helped create autonomous and very useful methods.
public class LinearEncoder extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor  climb= null;
    public DcMotor arm = null;
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;



    public void initializeHardware(){
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        rearRight = hardwareMap.get(DcMotor.class, "rr");
        rearLeft = hardwareMap.get(DcMotor.class, "rl");
        arm = hardwareMap.get(DcMotor.class, "arm");
        climb = hardwareMap.get(DcMotor.class, "climb");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initializeHardware();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        resetArmEncoders();
        resetDriveEncoders();
        resetElevatorEncoders();
        moveArmToPos(-250,0.5);

        double leftPower = 0.0;
        double rightPower = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setPower(leftPower);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setPower(rightPower);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




            if (gamepad2.y) {
                moveElevatorToPos(765);
            } else if (gamepad2.x) {
                moveElevatorToPos(0);
            } else if (gamepad2.dpad_left) {
                moveElevatorToPos(climb.getCurrentPosition() + 10);
            }
            if(gamepad2.b)
                moveArmToPos(-1200, 0.5);
            else if(gamepad2.a) {
                moveArmToPos(0, 0.59);
                //This runtIME IS THE ERROR UR LOOKING FOR
                sleep(500);
                moveArmToPos(-250, 0.5);
            }
            else if(gamepad2.left_bumper){
                moveArmToPos(0,0.59);
            }

            while(gamepad2.dpad_down){
                resetElevatorEncoders();
            }

            while(gamepad2.dpad_up){
                resetArmEncoders();
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Lift Position (Reset Elevator Encoder button: Start):", climb.getCurrentPosition());
            telemetry.addData("Arm Position (Reset Arm Encoder button: Mode):", arm.getCurrentPosition());
            telemetry.addData("Heading: ", getAngle());
            telemetry.update();
        }
    }
    //Move Arm to selected position
    public void moveArmToPos(int target, double power) {
        //Clip the power values so that it only goes from -1 to 1
        //power = Range.clip(power, -1, 1);

        if (arm != null) {
            // set motor to the correct mode (if necessary)
            if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // get current position
            int pos = arm.getCurrentPosition();

            // set where we want to move
            arm.setTargetPosition(target);

            // set the speed
            double sign = Math.signum(target - pos);
            arm.setPower(sign * power);
            if (arm.getCurrentPosition() < target) {
                arm.setPower(sign * power);
            } else if (arm.getCurrentPosition() > target) {
                arm.setPower(sign * power);
            } else {

            }
        }
    }

    /**
     * Reset Elevator motor encoders.
     */
    public void resetElevatorEncoders() {
        if (climb != null) {
            // stop motor
            climb.setPower(0);

            // send command
            climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
    }


    void moveRobotForwardToPos(float forward, double power) {
        // compute the number of ticks each robot has to move

        int fwd = (int)(288f * forward / (Math.PI * 6));
        power = Range.clip(power, -1, 1);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (rearLeft != null)           rearLeft.setTargetPosition(fwd);
        if (frontLeft != null)
            frontLeft.setTargetPosition(fwd);
        if (rearRight != null)
            rearRight.setTargetPosition(fwd);
        if (frontRight != null)
            frontRight.setTargetPosition(fwd);

        if (rearLeft != null)
            rearLeft.setPower(power);
        if (frontLeft != null)
            frontLeft.setPower(power);
        if (rearRight != null)
            rearRight.setPower(power);
        if (frontRight != null)
            frontRight.setPower(power);
    }

    /**
     * Reset arm motor encoders.
     */
    public void resetArmEncoders() {
        if (arm != null) {
            // stop motor
            arm.setPower(0);

            // send command
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
    }

    public void moveElevatorToPos(int target) {
        //Clip the power values so that it only goes from -1 to 1
        //power = Range.clip(power, -1, 1);

        if (climb != null) {
            // set motor to the correct mode (if necessary)
            if (climb.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // get current position
            int pos = climb.getCurrentPosition();

            // set where we want to move
            climb.setTargetPosition(target);

            // set the speed
            double sign = Math.signum(target - pos);
            climb.setPower(sign * 0.75);
            if (climb.getCurrentPosition() < target) {
                climb.setPower(sign * 0.75);
            } else if (climb.getCurrentPosition() > target) {
                climb.setPower(sign * 0.75);
            } else {

            }
        }
    }

    /**
     * Set the drive motors to the specified mode.
     *
     * @param mode the motor run mode
     */
    void setDriveMode(DcMotor.RunMode mode) {
        if (rearLeft != null)
            rearLeft.setMode(mode);
        if (frontLeft != null)
            frontLeft.setMode(mode);
        if (rearRight != null)
            rearRight.setMode(mode);
        if (frontRight != null)
            frontRight.setMode(mode);
    }
    /**
     * Set the power for both drive motors using the specified values.
     *
     * @param forward  the power level of the forward motion (+forward)
     * @param right the power level of the right motion (+right)
     * @param turn  the power level of the turn motion (+clockwise)
     */
    void setDrivePower(double forward, double right, double turn) {

        double leftFrontPower  = forward - right + turn;
        double leftRearPower   = forward + right + turn;
        double rightFrontPower = forward + right - turn;
        double rightRearPower  = forward - right - turn;

        //Clip the power values so that it only goes from -1 to 1
        leftFrontPower  = Range.clip(leftFrontPower, -1, 1);
        leftRearPower   = Range.clip(leftRearPower, -1, 1);
        rightFrontPower = Range.clip(rightFrontPower, -1, 1);
        rightRearPower  = Range.clip(rightRearPower, -1, 1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = false;//opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && (leftPower > 0 || rightPower > 0);

        if (!stop) {
            setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (rearLeft != null)
                rearLeft.setPower(leftRearPower);
            if (frontLeft != null)
                frontLeft.setPower(leftFrontPower);
            if (rearRight != null)
                rearRight.setPower(rightRearPower);
            if (frontRight != null)
                frontRight.setPower(rightFrontPower);
        }
    }

    void resetDriveEncoders() {
        if (rearLeft != null)
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (frontLeft != null)
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rearRight != null)
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (frontRight != null)
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // reset absolute variables
        resetAngle();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public double delay(double seconds) {
        double nanoToNormal = seconds * (10^4);
        double delayTime = getRuntime() + nanoToNormal;
        double waitFor = runtime.seconds() + delayTime;
        return waitFor;
    }




}
