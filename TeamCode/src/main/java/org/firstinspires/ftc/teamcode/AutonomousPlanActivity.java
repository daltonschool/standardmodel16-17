package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.Spinner;
import android.widget.TextView;

import org.firstinspires.ftc.teamcode.options.OptionManager;

import java.util.ArrayList;
import java.util.Arrays;

public class AutonomousPlanActivity extends AppCompatActivity implements CheckBox.OnCheckedChangeListener, AdapterView.OnItemSelectedListener {

    boolean isConfigValid = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_autonomous_plan);
        setTitle("Autonomous settings");

        OptionManager.init();

        // yes this is ugly but it's easier than a NumberPicker so TOO BAD
        ArrayAdapter<String> startDelayAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_dropdown_item, Arrays.asList("No start delay", "1 second", "2 seconds", "3 seconds", "4 seconds", "5 seconds", "6 seconds", "7 seconds", "8 seconds", "9 seconds", "10 seconds", "11 seconds", "12 seconds", "13 seconds", "14 seconds", "15 seconds"));
        ((Spinner)findViewById(R.id.settings_start_delay)).setAdapter(startDelayAdapter);

        ArrayAdapter<String> shootModeAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_dropdown_item, Arrays.asList("Two particles", "One particle", "No particles"));
        ((Spinner)findViewById(R.id.settings_shoot_mode)).setAdapter(shootModeAdapter);
        ((Spinner)findViewById(R.id.settings_shoot_mode)).setOnItemSelectedListener(this);

        ArrayAdapter<String> beaconModeAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_dropdown_item, Arrays.asList("Both beacons", "First beacon only", "No beacons"));
        ((Spinner)findViewById(R.id.settings_beacon_mode)).setAdapter(beaconModeAdapter);
        ((Spinner)findViewById(R.id.settings_beacon_mode)).setOnItemSelectedListener(this);

        ((CheckBox)findViewById(R.id.settings_cap_ball)).setOnCheckedChangeListener(this);

        updateInfoText();
    }

    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean val) {
        // TODO: find better way of doing this
        /*switch (compoundButton.getId()) {
            case R.id.settings_shots_only:
                OptionManager.currentOptions.shotsOnly = val;
                break;
        }*/

        OptionManager.save(OptionManager.currentOptions);
        updateInfoText();
    }

    @Override
    public void onItemSelected(AdapterView<?> adapterView, View view, int position, long id) {
        updateInfoText();
    }

    @Override
    public void onNothingSelected(AdapterView<?> adapterView) {
        updateInfoText();
    }

    public void updateInfoText() {
        int particleCount = 2 - ((Spinner)findViewById(R.id.settings_shoot_mode)).getSelectedItemPosition();
        int particlePoints = 15 * particleCount;

        int beaconCount = 2 - ((Spinner)findViewById(R.id.settings_beacon_mode)).getSelectedItemPosition();
        int beaconPoints = 30 * beaconCount;

        int capBallPoints = (((CheckBox)findViewById(R.id.settings_cap_ball)).isChecked() ? 5 : 0);

        int parkingPoints = (capBallPoints > 0 ? 5 : 0); // TODO: offer parking modes? right now we assume that capball == park in center

        int totalPoints = particlePoints + beaconPoints + capBallPoints + parkingPoints;

        ((TextView)findViewById(R.id.settings_estimated_points)).setText("Estimated score: " + Integer.toString(totalPoints) + " pts");

        isConfigValid = true;

        if (
                (capBallPoints > 0 && beaconCount != 2) // trying capball w/o beacons
                ) {
            isConfigValid = false;
        }

        TextView configInfo = ((TextView)findViewById(R.id.settings_config_info));
        if (isConfigValid) {
            configInfo.setText("Valid configuration");
            configInfo.setTextColor(Color.argb(255, 0, 200, 109));
        } else {
            configInfo.setText("Invalid configuration");
            configInfo.setTextColor(Color.RED);
        }
    }
}
