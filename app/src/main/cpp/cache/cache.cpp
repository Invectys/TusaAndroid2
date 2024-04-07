
#include "cache/cache.h"
#include "util/android_log.h"
#include <fstream>
#include <sstream>

void Cache::setAbsoluteCacheDirPath(const char *path) {
    absoluteCachePath = std::string(path);
}

std::string Cache::readCachedData(const char *filename, bool& exist) {
    exist = true;
    std::ifstream file(absoluteCachePath + '/' + filename);
    if(!file.is_open() || !file.good()) {
        exist = false;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    file.close();
    LOGI("Cache read");
    return content;
}

void Cache::cacheData(const char *filename, const std::string &data) {
    std::ofstream file(absoluteCachePath + '/' + filename);
    file << data;
    file.close();
    LOGI("Cache tile ", filename);
}

std::string Cache::getAbsoluteCachePath() {
    return absoluteCachePath;
}
