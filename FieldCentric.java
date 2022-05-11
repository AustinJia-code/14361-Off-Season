package org.firstinspires.ftc.teamcode;
//hello
import java.lang.Math;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Field Centric", group="DriveModes") //USE STINKY PRESET
public class FieldCentric extends OpMode {
    //39a5c0

    private DcMotor frontRight, frontLeft, backRight, backLeft, slide, intake;//Instantiating the DcMotors for the wheels
    private Servo bucket;
    private boolean heldB, slow, mode;//Instantiating a boolean that will be used for the arm lock
    private BNO055IMU imu;
    private double x, rx, y, power;//Instantiating the vars that will be used for the power and direction of the DcMotors
    private double frontRightPower, frontLeftPower, backRightPower, backLeftPower, heldAngle;//Instantiating the different power vars for the different DcMotors
    private double adjL = 0.8;
    private double adjR = 0.8; //adjustment double to account for unbalanced robots. Mecanum wheels will turn if this isnt here because they are silly
    BNO055IMU.Parameters imuParameters;
    Orientation angles;
    Acceleration gravity;
    @Override
    public void init() {
        

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        x = 0.0;
        y = 0.0;
        slow = false;
        heldB = false;
        mode = false;
        //Setting all of the power vars to the default value of 0.0
        frontRightPower = frontLeftPower = backRightPower = backLeftPower = 0.0;
        //Setting all of the DcMotors to their current state by talking to the expansion hub
        frontRight = hardwareMap.dcMotor.get("FR"); //1
        frontLeft = hardwareMap.dcMotor.get("FL"); //1
        backRight = hardwareMap.dcMotor.get("BR"); //1
        backLeft = hardwareMap.dcMotor.get("BL"); //1
        slide = hardwareMap.dcMotor.get("S"); //2
        intake = hardwareMap.dcMotor.get("IN");
        bucket = hardwareMap.servo.get("BUCKET");
        
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //DIFFERENT SO WE CAN HAVE THE ARM LOCK AT POSITION more details in AutoTemplate
        slide.setTargetPosition(0);
        slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucket.setPosition(1);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
    backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }
    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        x = gamepad1.left_stick_x;//Setting the x var to the current state of the gamepad1 left stick x value (this is the robots horizontal movement)
        rx = gamepad1.right_stick_x;//Setting the x2 var to the current state of the gamepad1 right stick x value (this is the robots rotational movement)
        y = -gamepad1.left_stick_y;//Setting the y var to the current state of the gamepad1 left stick y value (this is the robots vertical movement)
        power = 1; //sets power variables so we can adjust how fast the robot is moving (like a sens multiplier in a video game)
        //Setting the power and power2 to either normal speed or half speed based on the gamepad1 right bumper
        if(gamepad1.left_bumper && !heldB){
            heldB = true;
        }if(!gamepad1.left_bumper && heldB){
            slow = !slow;
            heldB = false;
            if(gamepad1.right_bumper){
                mode = !mode;
            }
        }
        if(slow){
            power /= 2;
        }
        if(gamepad1.right_bumper){
            power /= 3;
        }
        if(gamepad1.y){
            imu.initialize(imuParameters);
        }
        if(gamepad1.right_stick_y > 0 && slide.getCurrentPosition()-50 > 0){
            slide.setTargetPosition(slide.getCurrentPosition()-50);
        }else if(gamepad1.right_stick_y < 0 && slide.getCurrentPosition()+50 < 1250){
            slide.setTargetPosition(slide.getCurrentPosition()+50);
        }
        
        if(gamepad1.x) intake.setPower(1); //if dpad left is pressed, spin carousel slowly
        else if(gamepad1.b) intake.setPower(-1); //if right is pressed, spin carousel quickly
        else intake.setPower(0);
        
        if(slide.getCurrentPosition() < 360){
            bucket.setPosition(1-0.12*slide.getCurrentPosition()/100);
        }else if(gamepad1.right_trigger > 0){ //if right trigger is pressed, and the cap is not at it's max position
            bucket.setPosition(bucket.getPosition()-0.01); //move it slightly to that point
        }else if(gamepad1.left_trigger > 0){ //same as above
            bucket.setPosition(bucket.getPosition()+0.01);
        }
        
        
        if(!mode){
        	double deg = -angles.firstAngle;
        	double rad = deg * Math.PI/180; 
        	double temp = y * Math.cos(rad) + x * Math.sin(rad);
        	x = -y * Math.sin(rad) + x * Math.cos(rad);
        	 y = temp;
        }
        
        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) * power;
        double backLeftPower = (y - x + rx) * power;
        double frontRightPower = (y - x - rx) * power;
        double backRightPower = (y + x - rx) * power;
        double max = Math.max(1.0, Math.max(frontLeftPower, Math.max(backLeftPower, Math.max(frontRightPower, backRightPower))));
        //endregion
        if(gamepad1.a){
            frontLeftPower = 0.5;
            backLeftPower = 0.5;
            frontRightPower = 0.5;
            backRightPower = 0.5;
        }
        if(gamepad1.b){
            frontLeftPower = -0.5;
            backLeftPower = -0.5;
            frontRightPower = -0.5;
            backRightPower = -0.5;
        }
        frontLeft.setPower(frontLeftPower/max);
        backLeft.setPower(backLeftPower/max);
        frontRight.setPower(frontRightPower/max);
        backRight.setPower(backRightPower/max);
        telemetry.addData("Rot: ", angles.firstAngle);
        telemetry.addData("Bucket: ", bucket.getPosition());
        telemetry.addData("Slide Target: ", slide.getTargetPosition());
        telemetry.addData("Slide Cur: ", slide.getCurrentPosition());
    }
}