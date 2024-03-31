//package com.artem.tusaandroid;
//
//import android.opengl.GLSurfaceView;
//
//import javax.microedition.khronos.egl.EGL10;
//import javax.microedition.khronos.egl.EGLConfig;
//import javax.microedition.khronos.egl.EGLContext;
//import javax.microedition.khronos.egl.EGLDisplay;
//
//public class ContextFactory implements GLSurfaceView.EGLContextFactory {
//    @Override
//    public EGLContext createContext(EGL10 egl, EGLDisplay display, EGLConfig eglConfig) {
//        final int EGL_CONTEXT_CLIENT_VERSION = 0x3098;
//        int[] attrib_list = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL10.EGL_NONE };
//        EGLContext context = egl.eglCreateContext(display, eglConfig, EGL10.EGL_NO_CONTEXT, attrib_list);
//        return egl.eglCreateContext(display, eglConfig, EGL10.EGL_NO_CONTEXT, attrib_list);
//    }
//
//    @Override
//    public void destroyContext(EGL10 egl, EGLDisplay display, EGLContext context) {
//        egl.eglDestroyContext(display, context);
//    }
//}
