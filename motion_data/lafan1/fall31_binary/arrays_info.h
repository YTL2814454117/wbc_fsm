#ifndef ARRAYS_INFO_H
#define ARRAYS_INFO_H

// Generated from fallAndGetUp3_subject1.npz
// Contains information about converted arrays

#include <string>
#include <vector>
#include <map>

namespace ArraysInfo {

// Array metadata structure
struct ArrayMetadata {
    std::string name;
    std::vector<uint32_t> shape;
    char dtype_code;  // 'f'=float32, 'd'=float64, 'i'=int32, 'l'=int64
    uint32_t dtype_size;
    std::string filename;
};

// All arrays metadata
const std::map<std::string, ArrayMetadata> ARRAYS = {
    {"body_ang_vel_w", {"body_ang_vel_w", {5109, 30, 3}, 'f', 4, "body_ang_vel_w.bin"}},
    {"body_lin_vel_w", {"body_lin_vel_w", {5109, 30, 3}, 'f', 4, "body_lin_vel_w.bin"}},
    {"body_pos_w", {"body_pos_w", {5109, 30, 3}, 'f', 4, "body_pos_w.bin"}},
    {"body_quat_w", {"body_quat_w", {5109, 30, 4}, 'f', 4, "body_quat_w.bin"}},
    {"fps", {"fps", {1}, 'l', 8, "fps.bin"}},
    {"joint_pos", {"joint_pos", {5109, 29}, 'f', 4, "joint_pos.bin"}},
    {"joint_vel", {"joint_vel", {5109, 29}, 'f', 4, "joint_vel.bin"}},
}; // ARRAYS

// Total number of arrays
const size_t NUM_ARRAYS = 7;

} // namespace ArraysInfo

#endif // ARRAYS_INFO_H
