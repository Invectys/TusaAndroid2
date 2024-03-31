package com.artem.tusaandroid;

import android.opengl.GLSurfaceView;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.egl.EGLDisplay;

public class ConfigChooser implements GLSurfaceView.EGLConfigChooser {
    protected int redSize = 8;
    protected int greenSize = 8;
    protected int blueSize = 8 ;
    protected int alphaSize = 8;
    protected int depthSize = 32;
    protected int sampleSize = 4;
    protected int stencilSize = 0;
    protected int[] value = new int [1];

    @Override
    public EGLConfig chooseConfig(EGL10 egl, EGLDisplay display) {
        final int EGL_OPENGL_ES2_BIT = 4;
        int[] configAttributes = {
                EGL10.EGL_RED_SIZE, redSize,
                EGL10.EGL_GREEN_SIZE, greenSize,
                EGL10.EGL_BLUE_SIZE, blueSize,
                EGL10.EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
                EGL10.EGL_SAMPLES, sampleSize,
                EGL10.EGL_DEPTH_SIZE, depthSize,
                EGL10.EGL_STENCIL_SIZE, stencilSize,
                EGL10.EGL_NONE
        };
        int[] num_config = new int[1];
        egl.eglChooseConfig(display, configAttributes, null, 0, num_config);
        int numConfigs = num_config[0];
        EGLConfig[] configs = new EGLConfig[numConfigs];
        egl.eglChooseConfig(display, configAttributes, configs, numConfigs, num_config);
        return selectConfig(egl, display, configs);
    }

    public EGLConfig selectConfig(EGL10 egl, EGLDisplay display, EGLConfig[] configs)
    {
        for(EGLConfig config : configs)
        {
            int d = getConfigAttrib(egl, display, config, EGL10.EGL_DEPTH_SIZE, 0);
            int s = getConfigAttrib(egl, display, config, EGL10.EGL_GREEN_SIZE, 0);
            int r = getConfigAttrib(egl, display, config, EGL10.EGL_RED_SIZE,0);
            int g = getConfigAttrib(egl, display, config, EGL10.EGL_GREEN_SIZE, 0);
            int b = getConfigAttrib(egl, display, config, EGL10.EGL_BLUE_SIZE, 0);
            int a = getConfigAttrib(egl, display, config, EGL10.EGL_ALPHA_SIZE, 0);
            if (r == redSize && g == greenSize && b == blueSize && a == alphaSize && d >= depthSize && s >= stencilSize)
                return config;
        }
        return null;
    }

    private int getConfigAttrib(EGL10 egl, EGLDisplay display, EGLConfig config, int attribute, int defaultValue)
    {
        if (egl.eglGetConfigAttrib(display, config, attribute, value))
            return value[0];
        return defaultValue;
    }
}