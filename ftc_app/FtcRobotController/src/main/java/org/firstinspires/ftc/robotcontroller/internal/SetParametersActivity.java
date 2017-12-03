package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.text.InputFilter;
import android.text.InputType;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.NumberPicker;
import android.widget.Spinner;
import android.widget.TextView;
import com.qualcomm.ftcrobotcontroller.R;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by hsunx on 11/25/2017.
 */

public class SetParametersActivity extends Activity {

    // String array containing parameters that need to be set
    String[] values = {
            "ENUM",
            "start,RED_CLOSE,RED_FAR,BLUE_CLOSE,BLUE_FAR",
            "NUMBER",
            "red_threshold, 75",
            "NUMBER",
            "blue_threshold, 150"
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        final LinearLayout layout = (LinearLayout) findViewById(R.id.layout);

        for(int i = 0; i < values.length; i++) {
            // Parse string array and add components to LinearLayout accordingly
            String value = values[i];
            String[] args = null;
            if (value == "ENUM" || value == "NUMBER") {
                args = values[i+1].split(",");
                TextView name = new TextView(this);
                name.setText(args[0]);
                name.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT));
                layout.addView(name);
            }
            switch (value) {
                case "ENUM":
                    Spinner spinner = new Spinner(this);
                    String[] items = new String[args.length - 1];
                    for (int j = 1; j < args.length; j++) {
                        items[j-1] = args[j];
                    }
                    ArrayAdapter adapter = new ArrayAdapter(this, android.R.layout.simple_spinner_item, items);
                    adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                    spinner.setAdapter(adapter);
                    layout.addView(spinner);
                    break;
                case "NUMBER":
                    EditText numberInput = new EditText(this);
                    numberInput.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_VARIATION_NORMAL);
                    if (args.length == 2) {
                        numberInput.setText(args[1]);
                    }
                    layout.addView(numberInput);
                    break;
            }
        }

        // Returns inputted data

        Button saveButton = (Button) findViewById(R.id.button);
        saveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                StringBuilder sb = new StringBuilder();
                for(int i = 0; i < values.length; i++) {
                    String value = values[i];
                    View view = null;
                    String name = "";
                    if (value == "ENUM" || value == "NUMBER") {
                        view = layout.getChildAt(i + 1);
                        String[] args = values[i + 1].split(",");
                        name = args[0];
                    }

                    switch (value) {
                        case "ENUM":
                            Spinner spinner = (Spinner) view;
                            String selected = spinner.getSelectedItem().toString();
                            sb.append(name + ":" + selected + "\n");
                            break;
                        case "NUMBER":
                            EditText picker = (EditText) view;
                            sb.append(name + ":" + picker.getText().toString() + "\n");
                            break;
                    }
                }
                done(sb.toString());
            }
        });
    }

    final void done(String result) {
        Intent returnIntent = new Intent();
        returnIntent.putExtra("data", result);
        setResult(RESULT_OK, returnIntent);
        finish();
    }
}
