package com.artem.tusaandroid

import java.io.ByteArrayOutputStream
import java.net.HttpURLConnection
import java.net.URL


class RequestTile {
    fun request(zoom: Int, x: Int, y: Int): ByteArray {
        val base = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8,mapbox.mapbox-terrain-v2/"
        val token = "pk.eyJ1IjoiaW52ZWN0eXMiLCJhIjoiY2w0emRzYWx5MG1iMzNlbW91eWRwZzdldCJ9.EAByLTrB_zc7-ytI6GDGBw"
        val url = URL("$base$zoom/$x/$y.mvt?access_token=$token")
        val result = ByteArrayOutputStream()
        val urlConnection = url.openConnection() as HttpURLConnection
        urlConnection.inputStream.use { inputStream ->
            val buffer = ByteArray(1024)
            var length: Int
            while (inputStream.read(buffer).also { length = it } != -1) {
                result.write(buffer, 0, length)
            }
        }
        return result.toByteArray()
    }
}