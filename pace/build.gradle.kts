// Pace metronome for the human pace experiment (Tests/Human Tests): a single
// WebView showing src/main/assets/pace_metronome.html. Runs on a second device
// while the phone on the tripod records.
plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.example.pacemetronome"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.example.pacemetronome"
        minSdk = 24
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}
