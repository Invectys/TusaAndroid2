package com.artem.tusaandroid;

import android.content.res.AssetManager;

public class NativeLibrary {
    static {
        System.loadLibrary("tusaandroid");
    }
    public static native void render();
    public static native void onSurfaceChanged(int width, int height);
    public static native void surfaceCreated(AssetManager assetManager);
    public static native void noOpenGlContextInit(AssetManager assetManager, float appStartScaleFactor);
    public static native void setupCacheDirAbsolutePath(String path);
    public static native void drag(float dx, float dy);
    public static native void scale(float scaleFactor);
    public static native void doubleTap();
}
