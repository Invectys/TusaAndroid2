package com.artem.tusaandroid

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.artem.tusaandroid.databinding.ActivityMainBinding
import java.util.concurrent.Executors


class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    var executorService = Executors.newFixedThreadPool(1)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        NativeLibrary.setupCacheDirAbsolutePath(cacheDir.absolutePath)

//        executorService.submit {
//            val result = RequestTile().request(0,0,0)
//        }

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
    }

}