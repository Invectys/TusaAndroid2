//
// Created by Artem on 20.01.2024.
//

#include "network/request.h"
#include <fstream>
#include <curl/curl.h>
#include <stdio.h>
#include <string>
#include <android/log.h>
#include "map/tile.h"
#include "style/style.h"
#include <filesystem>
#include <vector_tile.hpp>

#define LOG_TAG "GL_ARTEM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

bool USE_CACHE_FOR_TILES = true;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

Tile* Request::loadVectorTile(int zoom, int x, int y) {
    std::string key = Tile::makeKey(zoom, x, y);
    bool exist;
    std::string cachedData = cache->readCachedData(key.c_str(), exist);
    if(exist && USE_CACHE_FOR_TILES) {
        return new Tile(zoom, x, y, new vtzero::vector_tile(cachedData));
    }

    std::string uri;
    std::string method = "GET";
    std::string arguments;
    std::string output;
    std::string api = "pk.eyJ1IjoiaW52ZWN0eXMiLCJhIjoiY2w0emRzYWx5MG1iMzNlbW91eWRwZzdldCJ9.EAByLTrB_zc7-ytI6GDGBw";
    std::string url = "http://api.mapbox.com/v4/mapbox.mapbox-streets-v8,mapbox.mapbox-terrain-v2/" + std::to_string(zoom) + "/" + std::to_string(x) + "/" + std::to_string(y) + ".mvt?access_token=" + api;
    const char* urlPtr = url.c_str();

    CURL *curl = curl_easy_init();
    CURLcode res;
    std::string readBuffer;
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, urlPtr);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "gzip");
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            throw std::runtime_error("curl_easy_perform() failed.");
        }
    } else {
        throw std::runtime_error("curl_easy_init() failed.");
    }
    curl_easy_cleanup(curl);

    cache->cacheData(key.c_str(), readBuffer);
    Tile* newTile = new Tile(
            zoom, x, y, new vtzero::vector_tile(readBuffer)
    );

    return newTile;
}

Request::Request(Cache *cache): cache(cache) {}