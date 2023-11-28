plugins {
    id("com.android.application")
}

android {
    namespace = "com.i8r441m.vio"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.i8r441m.vio"
        minSdk = 30
        targetSdk = 33
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
}

dependencies {

    implementation("androidx.fragment:fragment:1.6.1")
    implementation("com.google.android.material:material:1.10.0")
    implementation ("com.google.ar:core:1.33.0")
    implementation("com.google.ar.sceneform.ux:sceneform-ux:1.17.1")
    implementation ("com.google.protobuf:protobuf-java:3.25.1")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
}