package com.artem.tusaandroid

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView
import com.artem.tusaandroid.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        NativeLibrary.setupCacheDirAbsolutePath(cacheDir.absolutePath)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
    }
}