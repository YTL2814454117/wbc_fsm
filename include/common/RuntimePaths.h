#ifndef RUNTIME_PATHS_H
#define RUNTIME_PATHS_H

#include <cstdlib>
#include <string>

namespace RuntimePaths
{
inline std::string stripTrailingSlash(std::string path)
{
    while (path.size() > 1 && path.back() == '/')
        path.pop_back();
    return path;
}

inline bool isAbsolute(const std::string &path)
{
    return !path.empty() && path.front() == '/';
}

inline std::string root()
{
    const char *runtime_root = std::getenv("QIANER_G1_ROOT");
    if (runtime_root != nullptr && runtime_root[0] != '\0')
        return stripTrailingSlash(std::string(runtime_root));
    return stripTrailingSlash(std::string(PROJECT_ROOT_DIR));
}

inline std::string resolve(const std::string &path)
{
    if (path.empty() || isAbsolute(path))
        return path;
    return root() + "/" + path;
}
} // namespace RuntimePaths

#endif // RUNTIME_PATHS_H
