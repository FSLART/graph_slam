#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

#include <string>

class MapManager
{
public:
    MapManager();
    ~MapManager();
    static void load_map(const std::string& filename);
    static void save_map(const std::string& filename);
};

#endif // MAP_MANAGER_H_