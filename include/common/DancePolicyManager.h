#ifndef DANCE_POLICY_MANAGER_H
#define DANCE_POLICY_MANAGER_H

#include <cstddef>
#include <string>
#include <vector>

struct DancePolicyProfile
{
    std::string id;
    std::string name;
    std::string model_path;
    std::string motion_path;
    int start_idx = 0;
    int end_idx = -1;
    int pause_idx = 350;
    float safe_projgravity_threshold = 0.6f;
    bool debug = true;
    int debug_interval = 50;
    int return_to_amp_blend_frames = 20;
};

class DancePolicyManager
{
public:
    DancePolicyManager(const std::string &config_path, const std::string &project_root);

    const DancePolicyProfile &currentProfile() const;
    void selectNext();
    void selectPrev();
    void printCurrentProfile() const;
    std::string resolvePath(const std::string &path) const;
    std::size_t currentIndex() const { return _current_index; }
    std::size_t profileCount() const { return _profiles.size(); }

private:
    void loadConfig();

    std::string _config_path;
    std::string _project_root;
    std::vector<DancePolicyProfile> _profiles;
    std::size_t _current_index = 0;
};

#endif // DANCE_POLICY_MANAGER_H
