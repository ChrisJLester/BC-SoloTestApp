package com.beaumont.chrisj.spaghettidinner;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.o3dr.android.client.ControlTower;
import com.o3dr.android.client.Drone;
import com.o3dr.android.client.apis.ControlApi;
import com.o3dr.android.client.apis.GimbalApi;
import com.o3dr.android.client.apis.VehicleApi;
import com.o3dr.android.client.apis.solo.SoloCameraApi;
import com.o3dr.android.client.interfaces.DroneListener;
import com.o3dr.android.client.interfaces.TowerListener;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionResult;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.property.Altitude;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Battery;
import com.o3dr.services.android.lib.drone.property.Gps;
import com.o3dr.services.android.lib.drone.property.Home;
import com.o3dr.services.android.lib.drone.property.Speed;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.Type;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.model.AbstractCommandListener;
import com.o3dr.services.android.lib.model.SimpleCommandListener;
import com.o3dr.services.android.lib.util.MathUtils;

public class MainActivity extends AppCompatActivity implements SensorEventListener, TowerListener, DroneListener{

    //UI Components
    private ImageView compass_north, compass_drone;
    private TextView lblTowerConn, lblSoloConn, lblSoloBatt, lblSoloState, lblSoloMode, lblHomeLat, lblHomeLong,
            lblCurrentLat, lblCurrentLong, lblDistance, lblAlt, lblSpeed, lblYaw, lblTargetYaw;
    private Button btnConn, btnLaunch, btnLoadStream, btnRotateLeft, btnRotateRight, btnIncreaseAlt, btnDecreaseAlt;
    private ImageButton btnForward, btnRight, btnBackward, btnLeft;
    private TextureView stream_frame;

    //Compass Variables
    private SensorManager sensorManager;
    private Sensor gsensor;
    private Sensor msensor;
    private float[] mGravity = new float[3];
    private float[] mGeomagnetic = new float[3];
    private float azimuth = 0f;
    private float currectAzimuth = 0;

    //Drone Variables
    private ControlTower controlTower;
    private Drone drone;
    private int droneType = Type.TYPE_UNKNOWN;
    private final Handler handler = new Handler();
    private boolean isFlying;
    private double drone_yaw;
    private double target_yaw;
    private double yaw_before_action;

    //Drone movement Variables
    private int MOVEMENT_YAW;
    private int MOVEMENT_ALT;
    private int MOVEMENT_DEG;
    private float TURN_SPD;
    private int YAW_CHK_DUR;

    //Stream Variables
    private boolean stream_loaded;
    GimbalApi.GimbalOrientation orientation;
    public orientationListener ol;

    @Override
    public void onStart() {
        super.onStart();
        this.controlTower.connect(this);
    }

    @Override
    public void onStop() {
        super.onStop();
        if (this.drone.isConnected()) {
            this.drone.disconnect();
            updateConnectedButton(false);
        }
        this.controlTower.unregisterDrone(this.drone);
        this.controlTower.disconnect();
    }

    @Override
    protected void onResume() {
        super.onResume();
        sensorManager.registerListener(this, gsensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(this, msensor, SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this, gsensor);
        sensorManager.unregisterListener(this, msensor);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not in use
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        final float alpha = 0.97f;

        synchronized (this) {
            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

                mGravity[0] = alpha * mGravity[0] + (1 - alpha)
                        * event.values[0];
                mGravity[1] = alpha * mGravity[1] + (1 - alpha)
                        * event.values[1];
                mGravity[2] = alpha * mGravity[2] + (1 - alpha)
                        * event.values[2];
            }

            if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {

                mGeomagnetic[0] = alpha * mGeomagnetic[0] + (1 - alpha)
                        * event.values[0];
                mGeomagnetic[1] = alpha * mGeomagnetic[1] + (1 - alpha)
                        * event.values[1];
                mGeomagnetic[2] = alpha * mGeomagnetic[2] + (1 - alpha)
                        * event.values[2];

            }

            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity,
                    mGeomagnetic);
            if (success) {
                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                azimuth = (float) Math.toDegrees(orientation[0]); // orientation
                azimuth = (azimuth + 360) % 360;
                adjustCompass();
            }
        }
    }

    @Override
    public void onTowerConnected(){
        lblTowerConn.setText(R.string.tower_conn);
        this.controlTower.registerDrone(this.drone, this.handler);
        this.drone.registerDroneListener(this);
    }

    @Override
    public void onTowerDisconnected(){
        lblTowerConn.setText(R.string.tele_tower);
    }

    @Override
    public void onDroneConnectionFailed(ConnectionResult cr){
        makeToast("Drone Connection failed");
    }

    @Override
    public void onDroneServiceInterrupted(String s){
        makeToast("Drone service interrupted");
    }

    @Override
    public void onDroneEvent(String event, Bundle extras) {
        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                makeToast("Drone Connected");
                lblSoloConn.setText(R.string.solo_conn);
                updateConnectedButton(this.drone.isConnected());
                updateLaunchButton();
                break;
            case AttributeEvent.STATE_DISCONNECTED:
                makeToast("Drone Disconnected");
                lblSoloConn.setText(R.string.tele_solo);
                updateConnectedButton(this.drone.isConnected());
                updateLaunchButton();
                break;
            case AttributeEvent.STATE_UPDATED:
            case AttributeEvent.STATE_ARMING:
                updateLaunchButton();
                break;
            case AttributeEvent.STATE_VEHICLE_MODE:
                updateVehicleMode();
                break;
            case AttributeEvent.TYPE_UPDATED:
                Type newDroneType = this.drone.getAttribute(AttributeType.TYPE);
                if (newDroneType.getDroneType() != this.droneType) {
                    this.droneType = newDroneType.getDroneType();
                }
                break;
            case AttributeEvent.BATTERY_UPDATED:
                updateBattery();
                break;
            case AttributeEvent.ATTITUDE_UPDATED:
                updateAttitude();
                break;
            case AttributeEvent.SPEED_UPDATED:
                updateAltitude();
                updateSpeed();
                break;
            case AttributeEvent.HOME_UPDATED:
                updateHome();
                updateDistanceFromHome();
                break;
            default:
                break;
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        gsensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        msensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        MOVEMENT_YAW = 20;
        MOVEMENT_ALT = 10;
        MOVEMENT_DEG = 90;
        TURN_SPD = 0.5f;
        YAW_CHK_DUR = 5000;

        initUIComponents();

        this.controlTower = new ControlTower(getApplicationContext());
        this.drone = new Drone(getApplicationContext());

        makeToast("Welcome! :)");
    }


    //Launch controls actions
    //=========================================================================
    public void onBtnConn(View view) {
        if(this.drone.isConnected()) {
            this.drone.disconnect();
        } else {
            Bundle extraParams = new Bundle();
            extraParams.putInt(ConnectionType.EXTRA_UDP_SERVER_PORT, 14550); // Set default port to 14550

            ConnectionParameter connectionParams = new ConnectionParameter(ConnectionType.TYPE_UDP, extraParams, null);
            this.drone.connect(connectionParams);
        }
    }

    public void onBtnLaunch(View view) {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);

        if (vehicleState.isFlying()) {
            // Land
            VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_RTL);
        } else if (vehicleState.isArmed()) {
            // Take off
            ControlApi.getApi(this.drone).takeoff(15, new AbstractCommandListener() {
                @Override
                public void onSuccess() {

                }

                @Override
                public void onError(int executionError) {
                    makeToast("Failed to takeoff (Error)");
                }

                @Override
                public void onTimeout() {
                    makeToast("Failed to takeoff (Timeout)");
                }
            });
        } else if (!vehicleState.isConnected()) {
            // Connect
            makeToast("Connect to a drone first");
        } else if (vehicleState.isConnected() && !vehicleState.isArmed()){
            // Connected but not Armed
            VehicleApi.getApi(this.drone).arm(true);
        }
    }

    public void onBtnStop(View view){
        if(!isFlying) {
            ControlApi.getApi(this.drone).pauseAtCurrentLocation(new AbstractCommandListener() {
                @Override
                public void onSuccess() {

                }

                @Override
                public void onError(int executionError) {
                    read_executionError("Failed to pause", executionError);
                }

                @Override
                public void onTimeout() {
                    makeToast("Failed to pause (Timeout)");
                }
            });
        } else
            makeToast("You're not flying!");
    }

    private void updateConnectedButton(Boolean conn) {
        if (conn) {
            btnConn.setText(R.string.controls_disconn);
        } else {
            btnConn.setText(R.string.controls_conn);
        }
    }

    protected void updateLaunchButton() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);

        if (!this.drone.isConnected()) {
            btnLaunch.setVisibility(View.INVISIBLE);
        } else {
            btnLaunch.setVisibility(View.VISIBLE);
        }

        isFlying = vehicleState.isFlying();

        if (vehicleState.isFlying()) {
            // Land
            lblSoloState.setText(R.string.solo_state_flying);
            btnLaunch.setText(R.string.action_btn_land);
        } else if (vehicleState.isArmed()) {
            // Take off
            lblSoloState.setText(R.string.solo_state_armed);
            btnLaunch.setText(R.string.action_btn_takeoff);
        } else if (vehicleState.isConnected()){
            // Connected but not Armed
            lblSoloState.setText(R.string.solo_state_connected);
            btnLaunch.setText(R.string.action_btn_arm);
        }
    }


    //Flight Controls
    //=========================================================================
    public void onBtnForward(View view){
        moveDrone(0.0);
    }

    public void onBtnBackward(View view){
        moveDrone(180.0);
    }

    public void onBtnLeft(View view){
        moveDrone(270.0);
    }

    public void ontBtnRight(View view){
        moveDrone(90.0);
    }

    public void onBtnRotateRight(View view){
        yaw_before_action = drone_yaw;

        //Drone yaw goes from 0 to 180 and then -179 back to 0. This converts it to 0-360
        double current_yaw = (drone_yaw < 0 ? (180 + (180 - (-drone_yaw))) : drone_yaw);

        target_yaw = current_yaw + MOVEMENT_DEG;
        target_yaw = (target_yaw >= 360 ? (target_yaw - 360) : target_yaw);

        lblTargetYaw.setText(String.format("Target Yaw: %.2f", target_yaw));

        if(isFlying) {
            ControlApi.getApi(this.drone).turnTo((float)target_yaw, TURN_SPD, false, new AbstractCommandListener() {
                @Override
                public void onSuccess() {}
                @Override
                public void onError(int executionError) {
                    read_executionError("Failed to rotate", executionError);
                }
                @Override
                public void onTimeout() {
                    makeToast("Failed to rotate (timeout)");
                }

            });
        } else
            makeToast("You're not flying!");
    }

    public void onBtnRotateLeft(View view){
        yaw_before_action = drone_yaw;

        //Drone yaw goes from 0 to 180 and then -179 back to 0. This converts it to 0-360
        double current_yaw = (drone_yaw < 0 ? (180 + (180 - (-drone_yaw))) : drone_yaw);

        target_yaw = current_yaw - MOVEMENT_DEG;
        target_yaw = (target_yaw < 0 ? (target_yaw + 360) : target_yaw);

        lblTargetYaw.setText(String.format("Target Yaw: %.2f", target_yaw));

        if(isFlying) {
            ControlApi.getApi(this.drone).turnTo((float) target_yaw, -TURN_SPD, false, new AbstractCommandListener() {
                @Override
                public void onSuccess() {}

                @Override
                public void onError(int executionError) {
                    read_executionError("Failed to rotate", executionError);
                }

                @Override
                public void onTimeout() {
                    makeToast("Failed to rotate (timeout)");
                }

            });
        } else
            makeToast("You're not flying!");
    }

    public void onBtnIncreaseAltitude(View view){
        yaw_before_action = drone_yaw;

        if(isFlying) {
            Altitude alt = this.drone.getAttribute(AttributeType.ALTITUDE);
            ControlApi.getApi(this.drone).climbTo(alt.getAltitude() + MOVEMENT_ALT);
            check_yaw();
        } else
            makeToast("You're not flying!");
    }

    public void onBtnDecreaseAltitude(View view){
        yaw_before_action = drone_yaw;

        if(isFlying) {
            Altitude alt = this.drone.getAttribute(AttributeType.ALTITUDE);
            double target_alt = alt.getAltitude() - MOVEMENT_ALT;

            if (target_alt <= 0)
                makeToast("This will put the drone below the ground! Try landing");
            else {
                ControlApi.getApi(this.drone).climbTo(alt.getAltitude() - MOVEMENT_ALT);
                check_yaw();
            }
        } else
            makeToast("You're not flying!");
    }

    public void onBtnForceGuidedMode(View view){
        force_Guided_mode();
    }

    private void moveDrone(double bearing){
        if(isFlying) {
            yaw_before_action = drone_yaw;

            double target_bearing = bearing + drone_yaw;
            if (target_bearing >= 360)
                target_bearing = target_bearing - 360;

            LatLong current;
            try {
                Gps gps = this.drone.getAttribute(AttributeType.GPS);
                current = new LatLong(gps.getPosition().getLatitude(), gps.getPosition().getLongitude());
            } catch (Exception e) {
                current = new LatLong(54.068164, -2.801859);
            }

            LatLong target = MathUtils.newCoordFromBearingAndDistance(current, target_bearing, MOVEMENT_YAW);

            ControlApi.getApi(this.drone).goTo(target, true, new AbstractCommandListener() {
                @Override
                public void onSuccess() {
                    check_yaw();
                }

                @Override
                public void onError(int executionError) {
                    makeToast("Couldn't move (Error)");
                }

                @Override
                public void onTimeout() {
                    makeToast("Couldn't move (Timeout)");
                }
            });
        } else
            makeToast("You're not flying!");
    }


    //Stream Controls
    //=========================================================================
    public void onBtnLoadStream(View view){
        if(stream_loaded){
            stopVideoStream();
        } else {
            if(stream_frame.isAvailable()){
                makeToast("Stream available");
            } else {
                makeToast("Stream not available");
            }
            startVideoStream(new Surface(stream_frame.getSurfaceTexture()));
        }
    }

    public void onBtnPhoto(View view){
        SoloCameraApi.getApi(this.drone).takePhoto(new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                makeToast("Photo taken.");
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Photo error: ", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Timeout while trying to take the photo.");
            }
        });
    }

    public void onBtnRecord(View view){
        SoloCameraApi.getApi(drone).toggleVideoRecording(new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                makeToast("Video recording toggled.");
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Error toggling record: ", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Timeout while trying to toggle video recording.");
            }
        });
    }

    public void onBtnLookUp(View view){
        orientation = GimbalApi.getApi(this.drone).getGimbalOrientation();
        float pitch = orientation.getPitch();
        pitch = pitch + 5.0f;

        GimbalApi.getApi(this.drone).updateGimbalOrientation(pitch, orientation.getRoll(), orientation.getYaw(), ol);
    }

    public void onBtnLookDown(View view){
        orientation = GimbalApi.getApi(this.drone).getGimbalOrientation();
        float pitch = orientation.getPitch();
        pitch = pitch - 5.0f;

        GimbalApi.getApi(this.drone).updateGimbalOrientation(pitch, orientation.getRoll(), orientation.getYaw(), ol);
    }

    private void startVideoStream(Surface videoSurface) {
        SoloCameraApi.getApi(drone).startVideoStream(videoSurface, "", true, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                stream_loaded = true;
                btnLoadStream.setText(R.string.stream_stop);
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Cant load stream: ", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Timed out while attempting to start the video stream.");
            }
        });
        GimbalApi.getApi(this.drone).startGimbalControl(ol);
    }

    private void stopVideoStream() {
        SoloCameraApi.getApi(drone).stopVideoStream(new SimpleCommandListener() {
            @Override
            public void onSuccess() {
                stream_loaded = false;
                btnLoadStream.setText(R.string.stream_load);
            }
        });
        GimbalApi.getApi(this.drone).stopGimbalControl(ol);
    }


    //Telemetry updates
    //=========================================================================
    protected void updateVehicleMode() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        lblSoloMode.setText(getResources().getString(R.string.tele_update_mode, vehicleState.getVehicleMode()));
    }

    protected void updateAttitude(){
        Attitude droneAttitude = this.drone.getAttribute(AttributeType.ATTITUDE);
        drone_yaw = droneAttitude.getYaw();
        lblYaw.setText(String.format("Drone yaw: %.2f", droneAttitude.getYaw()));

        //Drone yaw goes from 0 to 180 and then -179 back to 0. This converts it to 0-360
        double current_yaw = (drone_yaw < 0 ? (180 + (180 - (-drone_yaw))) : drone_yaw);
        float target_yaw = azimuth + (float)current_yaw;
        compass_drone.setRotation(target_yaw);
    }

    protected void updateAltitude() {
        Altitude droneAltitude = this.drone.getAttribute(AttributeType.ALTITUDE);
        lblAlt.setText(getResources().getString(R.string.tele_update_spd, droneAltitude.getAltitude()));
    }

    protected void updateSpeed() {
        Speed droneSpeed = this.drone.getAttribute(AttributeType.SPEED);
        lblSpeed.setText(getResources().getString(R.string.tele_update_spd, droneSpeed.getGroundSpeed()));

        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
        lblCurrentLat.setText(String.format("Lat: %.2f", droneGps.getPosition().getLatitude()));
        lblCurrentLong.setText(String.format("Long: %.2f", droneGps.getPosition().getLongitude()));

        updateDistanceFromHome();
    }

    protected void updateHome(){
        Home droneHome = this.drone.getAttribute(AttributeType.HOME);
        lblHomeLat.setText(getResources().getString(R.string.tele_update_home_lat, droneHome.getCoordinate().getLatitude()));
        lblHomeLong.setText(getResources().getString(R.string.tele_update_home_long, droneHome.getCoordinate().getLongitude()));
    }

    protected void updateBattery(){
        Battery bat = this.drone.getAttribute(AttributeType.BATTERY);
        lblSoloBatt.setText(getResources().getString(R.string.tele_update_mode, bat.getBatteryRemain()));
    }

    protected void updateDistanceFromHome() {
        Altitude droneAltitude = this.drone.getAttribute(AttributeType.ALTITUDE);
        double vehicleAltitude = droneAltitude.getAltitude();
        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
        LatLong vehiclePosition = droneGps.getPosition();

        double distanceFromHome;

        if (droneGps.isValid()) {
            LatLongAlt vehicle3DPosition = new LatLongAlt(vehiclePosition.getLatitude(), vehiclePosition.getLongitude(), vehicleAltitude);
            Home droneHome = this.drone.getAttribute(AttributeType.HOME);
            distanceFromHome = distanceBetweenPoints(droneHome.getCoordinate(), vehicle3DPosition);
        } else {
            distanceFromHome = 0;
        }

        lblDistance.setText(getResources().getString(R.string.tele_update_batt, distanceFromHome));
    }

    public void onBtnSettings(View view){
        AlertDialog.Builder alertDialog = new AlertDialog.Builder(MainActivity.this);
        LayoutInflater inflater = getLayoutInflater();
        View convertView = inflater.inflate(R.layout.layout_settings, null);

        final EditText lblSettings_Yaw = (EditText)convertView.findViewById(R.id.lblSettings_Yaw);
        final EditText lblSettings_Alt = (EditText)convertView.findViewById(R.id.lblSettings_Alt);
        final EditText lblSettings_Deg = (EditText)convertView.findViewById(R.id.lblSettings_Deg);
        final EditText lblSettings_Spd = (EditText)convertView.findViewById(R.id.lblSettings_TurnSpd);
        final EditText lblSettings_Dur = (EditText)convertView.findViewById(R.id.lblSettings_ChkDur);

        lblSettings_Alt.setText(MOVEMENT_ALT + "");
        lblSettings_Deg.setText(MOVEMENT_DEG + "");
        lblSettings_Dur.setText(YAW_CHK_DUR / 1000 + "");
        lblSettings_Spd.setText((int)TURN_SPD * 100 + "");
        lblSettings_Yaw.setText(MOVEMENT_YAW + "");

        alertDialog.setView(convertView)
                .setTitle("Settings")
                .setNegativeButton("Cancel", null)
                .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        if (!(lblSettings_Alt.getText().toString().isEmpty()))
                            MOVEMENT_ALT = Integer.parseInt(lblSettings_Alt.getText().toString());

                        if (!(lblSettings_Deg.getText().toString().isEmpty()))
                            MOVEMENT_DEG = Integer.parseInt(lblSettings_Deg.getText().toString());

                        if (!(lblSettings_Dur.getText().toString().isEmpty()))
                            YAW_CHK_DUR = Integer.parseInt(lblSettings_Dur.getText().toString()) * 1000;

                        if (!(lblSettings_Spd.getText().toString().isEmpty()))
                            TURN_SPD = Float.parseFloat(lblSettings_Spd.getText().toString()) / 100;

                        if (!(lblSettings_Yaw.getText().toString().isEmpty()))
                            MOVEMENT_YAW = Integer.parseInt(lblSettings_Yaw.getText().toString());
                    }
                });
        alertDialog.show();
    }


    //Other
    //=========================================================================
    private void initUIComponents(){
        compass_north = (ImageView) findViewById(R.id.compass_north);
        compass_drone = (ImageView) findViewById(R.id.compass_drone);

        lblTowerConn = (TextView)findViewById(R.id.lblTowerConn);
        lblSoloConn = (TextView)findViewById(R.id.lblSoloConn);
        lblSoloBatt = (TextView)findViewById(R.id.lblSoloBatt);
        lblSoloState = (TextView)findViewById(R.id.lblSoloState);
        lblSoloMode = (TextView)findViewById(R.id.lblSoloMode);
        lblHomeLat = (TextView)findViewById(R.id.lblHomeLat);
        lblHomeLong = (TextView)findViewById(R.id.lblHomeLong);
        lblCurrentLat = (TextView)findViewById(R.id.lblCurrentLat);
        lblCurrentLong = (TextView)findViewById(R.id.lblCurrentLong);
        lblDistance = (TextView)findViewById(R.id.lblDistance);
        lblAlt = (TextView)findViewById(R.id.lblAlt);
        lblSpeed = (TextView)findViewById(R.id.lblSpeed);
        lblYaw = (TextView)findViewById(R.id.lblYaw);
        lblTargetYaw = (TextView)findViewById(R.id.lblTargetYaw);

        btnConn = (Button)findViewById(R.id.btnConn);
        btnLaunch = (Button)findViewById(R.id.btnLaunch);
        btnLoadStream = (Button)findViewById(R.id.btnLoadStream);
        btnRotateLeft = (Button)findViewById(R.id.btnRotateLeft);
        btnRotateRight = (Button)findViewById(R.id.btnRotateRight);
        btnIncreaseAlt = (Button)findViewById(R.id.btnIncreaseAlt);
        btnDecreaseAlt = (Button)findViewById(R.id.btnDecreaseAlt);

        btnForward = (ImageButton)findViewById(R.id.btnForward);
        btnRight = (ImageButton)findViewById(R.id.btnRight);
        btnBackward = (ImageButton)findViewById(R.id.btnBackward);
        btnLeft = (ImageButton)findViewById(R.id.btnLeft);

        stream_frame = (TextureView)findViewById(R.id.stream_frame);
        stream_frame.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
                makeToast("Video display is available.");
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
                return true;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surface) {

            }
        });
        stream_loaded = false;

        ol = new orientationListener();
    }

    private void adjustCompass() {
        if (compass_north == null) {
            return;
        }

        Animation an = new RotateAnimation(-currectAzimuth, -azimuth, Animation.RELATIVE_TO_SELF,
                0.5f, Animation.RELATIVE_TO_SELF, 0.5f);
        currectAzimuth = azimuth;

        an.setDuration(500);
        an.setRepeatCount(0);
        an.setFillAfter(true);

        compass_north.startAnimation(an);
    }

    protected double distanceBetweenPoints(LatLongAlt pointA, LatLongAlt pointB) {
        if (pointA == null || pointB == null) {
            return 0;
        }
        double dx = pointA.getLatitude() - pointB.getLatitude();
        double dy  = pointA.getLongitude() - pointB.getLongitude();
        double dz = pointA.getAltitude() - pointB.getAltitude();
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    private void makeToast(String message) {
        Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
    }

    private void force_Guided_mode(){
        VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_GUIDED);
    }

    private void read_executionError(String msg, int error){
        if (error == CommandExecutionError.COMMAND_DENIED)
            makeToast(msg + ": Command Denied");
        else if (error == CommandExecutionError.COMMAND_FAILED)
            makeToast(msg + ": Command Failed");
        else if (error == CommandExecutionError.COMMAND_TEMPORARILY_REJECTED)
            makeToast(msg + ": Command rejected");
        else if (error == CommandExecutionError.COMMAND_UNSUPPORTED)
            makeToast(msg + ": unsupported");
        else
            makeToast(msg + ": Error didn't match");
    }

    private void check_yaw(){
        toggle_buttons(false);
        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            public void run() {
                if(yaw_before_action != drone_yaw)
                    rotate();
                toggle_buttons(true);
            }
        }, YAW_CHK_DUR);
    }

    private void rotate(){
        ControlApi.getApi(this.drone).turnTo((float)yaw_before_action, TURN_SPD, false, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Failed to rotate", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Failed to rotate (timeout)");
            }
        });
    }

    public class orientationListener implements GimbalApi.GimbalOrientationListener{
        @Override
        public void onGimbalOrientationUpdate(GimbalApi.GimbalOrientation orientation) {}

        @Override
        public void onGimbalOrientationCommandError(int error) {
            if (error == CommandExecutionError.COMMAND_DENIED)
                makeToast("Gimball error: Command Denied");
            else if (error == CommandExecutionError.COMMAND_FAILED)
                makeToast("Gimball error: Command Failed");
            else if (error == CommandExecutionError.COMMAND_TEMPORARILY_REJECTED)
                makeToast("Gimball error: Command rejected");
            else if (error == CommandExecutionError.COMMAND_UNSUPPORTED)
                makeToast("Gimball error: unsupported");
            else
                makeToast("Error didn't match");
        }
    }

    private void toggle_buttons(boolean b){
        btnRotateLeft.setEnabled(b);
        btnRotateRight.setEnabled(b);
        btnIncreaseAlt.setEnabled(b);
        btnDecreaseAlt.setEnabled(b);

        btnForward.setEnabled(b);
        btnRight.setEnabled(b);
        btnBackward.setEnabled(b);
        btnLeft.setEnabled(b);
    }
}
