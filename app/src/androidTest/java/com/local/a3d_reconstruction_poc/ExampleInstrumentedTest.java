package com.local.a3d_reconstruction_poc;

import android.content.Context;
import android.support.test.InstrumentationRegistry;
import android.support.test.runner.AndroidJUnit4;

import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.IOException;
import java.io.InputStream;

import static org.junit.Assert.*;

/**
 * Instrumented test, which will execute on an Android device.
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
@RunWith(AndroidJUnit4.class)
public class ExampleInstrumentedTest {
    @Test
    public void useAppContext() {
        // Context of the app under test.
        Context appContext = InstrumentationRegistry.getTargetContext();

        assertEquals("com.local.a3d_reconstruction_poc", appContext.getPackageName());
    }

    @Test
    public void readImage() throws IOException {
        Context testContext = InstrumentationRegistry.getInstrumentation().getContext();
        InputStream testInput = testContext.getAssets().open("Ace_0.png");
    }

    @Test
    public void createReconstructor(){
        Context testContext = InstrumentationRegistry.getInstrumentation().getContext();
        Reconstructor r = new Reconstructor(testContext.getAssets());
        r.match("Ace_0.png", "Ace_1.png");
    }
}
