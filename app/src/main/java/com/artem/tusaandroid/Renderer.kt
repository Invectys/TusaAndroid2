package com.artem.tusaandroid

import android.content.res.AssetManager
import android.opengl.GLSurfaceView
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10


class Renderer(private val _assetManager: AssetManager) : GLSurfaceView.Renderer {
    override fun onSurfaceCreated(gl: GL10, config: EGLConfig) {
        NativeLibrary.surfaceCreated(_assetManager)
    }

    override fun onSurfaceChanged(gl: GL10, width: Int, height: Int) {
        NativeLibrary.onSurfaceChanged(width, height)
    }

    override fun onDrawFrame(gl: GL10) {
        NativeLibrary.render()
    }
}
