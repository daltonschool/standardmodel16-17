package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.ClassFactory_SM;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

// yes I know that technically vuforia isn't a sensor
// I don't really care though
public class Vuforia extends Sensor{
    private List<VuforiaTrackable> allTrackables;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftcTrackables;

    public VuforiaTrackable wheels;
    public VuforiaTrackable tools;
    public VuforiaTrackable legos;
    public VuforiaTrackable gears;

    private OpenGLMatrix lastLocation = null;

    @Override
    public boolean ping() {
        return true;
    }

    @Override
    public byte firmwareRevision() {
        return 0x01;
    }

    @Override
    public byte manufacturer() {
        return 0x42;
    }

    @Override
    public byte sensorIDCode() {
        return 0x00;
    }

    @Override
    public String name() {
        return "Vuforia";
    }

    @Override
    public String uniqueName() {
        return name();
    }

    public void init() {
        // vuforia init
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = "Aeqqx9n/////AAAAGU44IlIke0wcpp2TXZIm0doq2mr4uV5sFkonVd69btVkAHlcthh2lKkMMI+n0pvfyHG/1YVon/+hvr2sJ14bJp3HFifDm0EDP1lJ0B26oSFaShv339Snwjk53VLnXiIAxRu6ys9uovyitz8dlnnT8j6UHSRV1elViHriLiSJt9URKaUhoe0I0a+0XElImXIuZXN7p8NMMP/LIPK3bHYt3CIMIGQ4fSs1+4/06pqI06ijwsH1SIIZn0tiB4199YwyqLfea3Wi+Tsnwm3IkOhgWCy3JeHiCTs43EmciCH0ldF+2N/XuoFFMMPqe/81vMhdHWuWuQFPtXDK7wYrLNFqZ32YTGyKkhyFaejloP4No76F";
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory_SM.createVuforiaLocalizer_SM(vuforiaParameters);

        // vuforia trackables
        ftcTrackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        wheels = ftcTrackables.get(0);
        wheels.setName("Wheels");

        tools = ftcTrackables.get(1);
        tools.setName("Tools");

        legos = ftcTrackables.get(2);
        legos.setName("Legos");

        gears = ftcTrackables.get(3);
        gears.setName("Gears");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ftcTrackables);

        // units
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18f * mmPerInch;
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        // place beacons on field
        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, -(2*12*mmPerInch), 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsLocationOnField);

        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, (2*12*mmPerInch), 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsLocationOnField);

        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                .translation(-(2*12*mmPerInch), mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosLocationOnField);

        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                .translation((2*12*mmPerInch), mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);

        // phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation((mmBotWidth/2) + 50 ,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));

        // inform listeners about phone
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
    }

    public void start() {
        // activate vuforia dataset
        ftcTrackables.activate();
    }

    @Override
    public void update() {
        for (VuforiaTrackable trackable : allTrackables) {
            //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
            //((VuforiaTrackableDefaultListener)trackable.getListener()).
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
    }

    public boolean hasLocation() {
        return lastLocation != null;
    }

    public VectorF getLocation() {
        return lastLocation.getTranslation();
    }

    public String getLocationAsString() {
        return lastLocation.formatAsTransform();
    }

    public Orientation getOrientation() {
        return Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }
}
