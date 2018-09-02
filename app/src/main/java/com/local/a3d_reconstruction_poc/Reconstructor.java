package com.local.a3d_reconstruction_poc;

import android.content.res.AssetManager;

public class Reconstructor {

    static {
        System.loadLibrary("native-lib");
    }

    private AssetManager mgr;

    public Reconstructor(AssetManager am)
    {
        mgr = am;
    }


    public void match(String a, String b) {
        matchNative(mgr, a, b);
    }

    private static native void matchNative(AssetManager mgr, String a, String b);
}
