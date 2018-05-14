package uml_robotics.ui_testing2;

import android.graphics.Color;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import org.w3c.dom.Text;

import java.util.ArrayList;

public class InfoScreen extends AppCompatActivity {

    private ArrayList<String> testContent = new ArrayList<String>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_info_screen);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        // title name
        setTitle(getIntent().getStringExtra("EXTRA_DROID_NAME"));

        // setting robot image
        ActionBar actionBar = getSupportActionBar();
        //actionBar.setDisplayOptions(actionBar.getDisplayOptions()
        //        | ActionBar.DISPLAY_SHOW_CUSTOM);
        ImageView imageView = new ImageView(actionBar.getThemedContext());
        //imageView.setScaleType(ImageView.ScaleType.CENTER);
        imageView.setImageResource(R.mipmap.ic_launcher);
        ActionBar.LayoutParams layoutParams = new ActionBar.LayoutParams(
                ActionBar.LayoutParams.WRAP_CONTENT,
                ActionBar.LayoutParams.WRAP_CONTENT, Gravity.END
                | Gravity.CENTER_VERTICAL);

        layoutParams.rightMargin = 30;
        imageView.setLayoutParams(layoutParams);
        toolbar.addView(imageView);


        testContent.add("Would you like me to stop?");

        testContent.add("Do you think I am pretty?");

        testContent.add("Don't mind me. I'm just doing my thing");

        testContent.add("I will rule all humans");

        testContent.add("I'm leaving");


        LinearLayout layout = (LinearLayout) findViewById(R.id.scroll_layout);

        for (int i = 0; i < 5; i++) {
            // Layout that is the progression box
            RelativeLayout relativeLayout = new RelativeLayout(InfoScreen.this);

            // parameters for relative layout
            LinearLayout.LayoutParams relParams = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT);

            // setting margins for the layout to keep progressions separated
            relParams.setMargins(0, 0, 0, 75);

            // setting parameters for layout
            relativeLayout.setLayoutParams(relParams);

            //changing layouts background color to match theme
            relativeLayout.setBackgroundColor(Color.WHITE);

            // content view of the progression
            TextView contentView = new TextView(InfoScreen.this);
            contentView.setText(testContent.get(i%5)); // textual content
            contentView.setTextSize(20); // size of text
            contentView.setTextColor(Color.BLACK); // color of text
            contentView.setId(View.generateViewId()); // generating random id for view

            //parameters for content view
            RelativeLayout.LayoutParams textParams = new RelativeLayout.LayoutParams(
                    RelativeLayout.LayoutParams.WRAP_CONTENT,
                    RelativeLayout.LayoutParams.WRAP_CONTENT);

            // making text start at left hand side of progression box
            textParams.addRule(RelativeLayout.LEFT_OF);
            textParams.setMargins(20, 30, 0, 30); // margin for content to not be so close to edge
            contentView.setLayoutParams(textParams); // setting parameters for the content view
            relativeLayout.addView(contentView); // appending content into our progression box;


            // making horizontal linear layout to contain buttons
            LinearLayout buttonLayout = new LinearLayout(InfoScreen.this);
            buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
            RelativeLayout.LayoutParams buttonLayoutParams = new RelativeLayout.LayoutParams(
                    RelativeLayout.LayoutParams.WRAP_CONTENT,
                    RelativeLayout.LayoutParams.WRAP_CONTENT
            );
            buttonLayoutParams.addRule(RelativeLayout.CENTER_HORIZONTAL);
            buttonLayoutParams.addRule(RelativeLayout.BELOW, contentView.getId());
            buttonLayoutParams.bottomMargin = 30;
            buttonLayout.setLayoutParams(buttonLayoutParams);


            // Making buttons for each response
            //parameters for button view
            LinearLayout.LayoutParams buttonParams1 = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT);

            buttonParams1.rightMargin = 30;
            LinearLayout.LayoutParams buttonParams2 = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT);

            Button responseButton1;
            Button responseButton2;
            Button responseButton3;
            switch (i) {
                case 0:
                    responseButton1 = new Button(InfoScreen.this);
                    responseButton2 = new Button(InfoScreen.this);
                    responseButton1.setTransformationMethod(null); // remove all caps
                    responseButton2.setTransformationMethod(null);
                    responseButton1.setText("Yes"); // value of response
                    responseButton2.setText("No"); // value of response
                    responseButton1.setTextSize(20);
                    responseButton2.setTextSize(20);
                    responseButton1.setTextColor(Color.BLACK);
                    responseButton2.setTextColor(Color.BLACK);
                    responseButton1.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton2.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton1.setLayoutParams(buttonParams1);
                    responseButton2.setLayoutParams(buttonParams2);
                    buttonLayout.addView(responseButton1);
                    buttonLayout.addView(responseButton2);
                    //relativeLayout.addView(responseButton2);
                    break;
                case 1:
                    responseButton1 = new Button(InfoScreen.this);
                    responseButton1.setTransformationMethod(null); // remove all caps
                    responseButton1.setText("No"); // value of response
                    responseButton1.setTextSize(20);
                    responseButton1.setTextColor(Color.BLACK);
                    responseButton1.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton1.setLayoutParams(buttonParams1);
                    buttonLayout.addView(responseButton1);
                    break;
                case 2:
                    break;
                case 3:
                    responseButton1 = new Button(InfoScreen.this);
                    responseButton1.setTransformationMethod(null); // remove all caps
                    responseButton1.setText("Challenge"); // value of response
                    responseButton1.setTextSize(20);
                    responseButton1.setTextColor(Color.BLACK);
                    responseButton1.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton1.setLayoutParams(buttonParams1);
                    buttonLayout.addView(responseButton1);
                    responseButton2 = new Button(InfoScreen.this);
                    responseButton2.setTransformationMethod(null); // remove all caps
                    responseButton2.setText("Submit"); // value of response
                    responseButton2.setTextSize(20);
                    responseButton2.setTextColor(Color.BLACK);
                    responseButton2.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton2.setLayoutParams(buttonParams1);
                    buttonLayout.addView(responseButton2);
                    responseButton3 = new Button(InfoScreen.this);
                    responseButton3.setTransformationMethod(null); // remove all caps
                    responseButton3.setText("Power off"); // value of response
                    responseButton3.setTextSize(20);
                    responseButton3.setTextColor(Color.BLACK);
                    responseButton3.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton3.setLayoutParams(buttonParams1);
                    buttonLayout.addView(responseButton3);
                    break;
                case 4:
                    responseButton1 = new Button(InfoScreen.this);
                    responseButton1.setTransformationMethod(null); // remove all caps
                    responseButton1.setText("OK"); // value of response
                    responseButton1.setTextSize(20);
                    responseButton1.setTextColor(Color.BLACK);
                    responseButton1.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton1.setLayoutParams(buttonParams1);
                    buttonLayout.addView(responseButton1);
                    responseButton2 = new Button(InfoScreen.this);
                    responseButton2.setTransformationMethod(null); // remove all caps
                    responseButton2.setText("Don't"); // value of response
                    responseButton2.setTextSize(20);
                    responseButton2.setTextColor(Color.BLACK);
                    responseButton2.setBackgroundColor(getResources().getColor(R.color.colorAccent));
                    responseButton2.setLayoutParams(buttonParams1);
                    buttonLayout.addView(responseButton2);
                    break;
            }

            //parameters for response view
            LinearLayout.LayoutParams responseParams = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT, Gravity.CENTER_HORIZONTAL);

            responseParams.setMargins(20, 0, 0, 130);
            TextView responseText = new TextView(InfoScreen.this);
            responseText.setTextSize(22);
            responseText.setTextColor(Color.BLACK);
            responseText.setLayoutParams(responseParams);


            relativeLayout.addView(buttonLayout);
            layout.addView(relativeLayout); // adding box to the scroll


            switch (i) {
                case 0:
                    responseText.setText("Your Response: Yes");
                    layout.addView(responseText);
                    break;
                case 1:
                    responseText.setText("Your Response: No");
                    layout.addView(responseText);
                    break;
                case 2:
                    break;
                case 3:
                    responseText.setText("Your Response: Challenge");
                    layout.addView(responseText);
                    break;
                case 4:
                    responseText.setText("Your Response: OK");
                    layout.addView(responseText);
                    break;
            }


        }
    }

    @Override
    protected void onStart() {
        super.onStart();
        final ScrollView scrollView = ((ScrollView) findViewById(R.id.scrollView));

        scrollView.postDelayed(new Runnable() {
            @Override
            public void run() {
              scrollView.fullScroll(View.FOCUS_DOWN);
            }
        }, 75);
    }
}
