#ifndef TUSA_CACHE_H
#define TUSA_CACHE_H

#include <string>

class Cache {
public:
    void setAbsoluteCacheDirPath(const char* path);
    std::string readCachedData(const char* filename, bool& exist);
    void cacheData(const char* filename, const std::string& data);
    std::string getAbsoluteCachePath();
private:
    std::string absoluteCachePath;
};


#endif