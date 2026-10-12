package com.example.pacemetronome;

import android.app.Activity;
import android.media.AudioManager;
import android.os.Bundle;
import android.view.WindowManager;
import android.webkit.WebView;

/** Hosts the metronome page; all of the logic is in assets/pace_metronome.html. */
public class MainActivity extends Activity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setVolumeControlStream(AudioManager.STREAM_MUSIC);
        WebView web = new WebView(this);
        web.getSettings().setJavaScriptEnabled(true);
        web.getSettings().setMediaPlaybackRequiresUserGesture(false);
        web.loadUrl("file:///android_asset/pace_metronome.html");
        setContentView(web);
    }
}
