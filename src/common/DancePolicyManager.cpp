#include "common/DancePolicyManager.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

DancePolicyManager::DancePolicyManager(const std::string &config_path, const std::string &project_root)
    : _config_path(config_path), _project_root(project_root)
{
    loadConfig();
}

void DancePolicyManager::loadConfig()
{
    std::ifstream config_file(_config_path);
    if (!config_file.is_open())
    {
        throw std::runtime_error("Cannot open dance policy config: " + _config_path);
    }

    json config = json::parse(config_file);
    if (!config.contains("dances") || !config["dances"].is_array() || config["dances"].empty())
    {
        throw std::runtime_error("Dance policy config must contain a non-empty dances array");
    }

    _profiles.clear();
    for (const auto &item : config["dances"])
    {
        DancePolicyProfile profile;
        profile.id = item.at("id").get<std::string>();
        profile.name = item.value("name", profile.id);
        profile.model_path = item.at("model_path").get<std::string>();
        profile.motion_path = item.at("motion_path").get<std::string>();
        profile.start_idx = item.value("start_idx", 0);
        profile.end_idx = item.value("end_idx", -1);
        profile.pause_idx = item.value("pause_idx", 350);
        profile.safe_projgravity_threshold = item.value("safe_projgravity_threshold", 0.6f);
        profile.debug = item.value("debug", true);
        profile.debug_interval = std::max(1, item.value("debug_interval", 50));
        profile.return_to_amp_blend_frames = std::max(1, item.value("return_to_amp_blend_frames", 20));
        _profiles.push_back(profile);
    }

    const std::string default_dance = config.value("default_dance", _profiles.front().id);
    _current_index = 0;
    for (std::size_t i = 0; i < _profiles.size(); ++i)
    {
        if (_profiles[i].id == default_dance)
        {
            _current_index = i;
            break;
        }
    }

    std::cout << "[DancePolicy] Loaded " << _profiles.size()
              << " dance profile(s) from " << _config_path << std::endl;
    printCurrentProfile();
}

const DancePolicyProfile &DancePolicyManager::currentProfile() const
{
    if (_profiles.empty())
    {
        throw std::runtime_error("No dance profiles loaded");
    }
    return _profiles[_current_index];
}

void DancePolicyManager::selectNext()
{
    if (_profiles.empty())
        return;
    _current_index = (_current_index + 1) % _profiles.size();
    printCurrentProfile();
}

void DancePolicyManager::selectPrev()
{
    if (_profiles.empty())
        return;
    _current_index = (_current_index + _profiles.size() - 1) % _profiles.size();
    printCurrentProfile();
}

void DancePolicyManager::printCurrentProfile() const
{
    const auto &profile = currentProfile();
    std::cout << "[DancePolicy] Selected [" << (_current_index + 1) << "/" << _profiles.size()
              << "] id=" << profile.id
              << " name=\"" << profile.name << "\""
              << " model=" << profile.model_path
              << " motion=" << profile.motion_path
              << " frames=" << profile.start_idx << "->" << profile.end_idx
              << " return_to_amp_blend_frames=" << profile.return_to_amp_blend_frames
              << std::endl;
}

std::string DancePolicyManager::resolvePath(const std::string &path) const
{
    if (path.empty())
        return path;
    if (path.front() == '/')
        return path;
    return _project_root + "/" + path;
}
