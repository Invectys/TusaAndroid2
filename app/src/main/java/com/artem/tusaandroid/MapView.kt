package com.artem.tusaandroid

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Matrix
import android.graphics.PointF
import android.opengl.GLSurfaceView
import android.util.AttributeSet
import android.view.GestureDetector
import android.view.GestureDetector.SimpleOnGestureListener
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.ScaleGestureDetector.SimpleOnScaleGestureListener


class MapView : GLSurfaceView {
    private var scaleGestureDetector: ScaleGestureDetector? = null
    private var gestureDetector: GestureDetector? = null

    // Зум при запуске от 0 до 19
    private val onAppStartMapZoom = 3

    private val scaleShift = 1.0f
    private val maxScale = 19f + scaleShift
    private var scaleFactor = onAppStartMapZoom + scaleShift
    private var lastFocusX = 0f
    private var lastFocusY = 0f
    private val translate = PointF(0f, 0f)

    constructor(context: Context) : super(context) {}
    constructor(context: Context, attributes: AttributeSet) : super(context, attributes) {}

    init {
        // Чтобы при старте синхронизировать C++ и Java состояния карты
        NativeLibrary.noOpenGlContextInit(resources.assets, scaleFactor)

        setEGLContextClientVersion(2)
        setRenderer(Renderer(resources.assets))

        scaleGestureDetector = ScaleGestureDetector(context, ScaleListener())
        gestureDetector = GestureDetector(context, GestureListener())
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleGestureDetector!!.onTouchEvent(event)
        gestureDetector!!.onTouchEvent(event)
        return true
    }

    private inner class ScaleListener : SimpleOnScaleGestureListener() {
        override fun onScale(detector: ScaleGestureDetector): Boolean {
            scaleFactor *= detector.scaleFactor
            scaleFactor = scaleShift.coerceAtLeast(scaleFactor.coerceAtMost(maxScale))
            NativeLibrary.scale(scaleFactor)
            return true
        }

        override fun onScaleBegin(detector: ScaleGestureDetector): Boolean {
            lastFocusX = detector.focusX
            lastFocusY = detector.focusY
            return true
        }
    }

    private inner class GestureListener : SimpleOnGestureListener() {
        override fun onScroll(
            e1: MotionEvent?,
            e2: MotionEvent,
            distanceX: Float,
            distanceY: Float
        ): Boolean {
            translate.x -= distanceX
            translate.y -= distanceY
            NativeLibrary.drag(distanceX, distanceY)
            return true
        }

        override fun onDoubleTap(e: MotionEvent): Boolean {
            NativeLibrary.doubleTap()
            return true
        }
    }
}