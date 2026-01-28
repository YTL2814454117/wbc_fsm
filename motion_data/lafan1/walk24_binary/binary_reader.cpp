// Sample C++ code for reading binary array files
// Generated from walk2_subject4.npz

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include "arrays_info.h"

class BinaryArrayReader {
public:
    // Read array from binary file
    template<typename T>
    static bool readArray(const std::string& filepath, std::vector<T>& data, std::vector<uint32_t>& shape) {
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filepath << std::endl;
            return false;
        }
        
        // Read and verify magic number
        char magic[4];
        file.read(magic, 4);
        if (std::memcmp(magic, "NPZ\0", 4) != 0) {
            std::cerr << "Invalid file format: " << filepath << std::endl;
            return false;
        }
        
        // Read number of dimensions
        uint32_t ndims;
        file.read(reinterpret_cast<char*>(&ndims), sizeof(uint32_t));
        
        // Read dimensions
        shape.resize(ndims);
        file.read(reinterpret_cast<char*>(shape.data()), ndims * sizeof(uint32_t));
        
        // Read data type info
        uint32_t dtype_size;
        file.read(reinterpret_cast<char*>(&dtype_size), sizeof(uint32_t));
        
        char dtype_code;
        file.read(&dtype_code, 1);
        
        // Skip reserved bytes
        file.seekg(3, std::ios::cur);
        
        // Calculate total number of elements
        size_t total_elements = 1;
        for (uint32_t dim : shape) {
            total_elements *= dim;
        }
        
        // Read data
        data.resize(total_elements);
        file.read(reinterpret_cast<char*>(data.data()), total_elements * sizeof(T));
        
        if (!file.good()) {
            std::cerr << "Error reading data from file: " << filepath << std::endl;
            return false;
        }
        
        return true;
    }
    
    // Print array information
    template<typename T>
    static void printArrayInfo(const std::string& name, const std::vector<T>& data, const std::vector<uint32_t>& shape) {
        std::cout << "Array: " << name << std::endl;
        std::cout << "  Shape: (";
        for (size_t i = 0; i < shape.size(); ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << shape[i];
        }
        std::cout << ")" << std::endl;
        std::cout << "  Elements: " << data.size() << std::endl;
        std::cout << "  Data type size: " << sizeof(T) << " bytes" << std::endl;
        
        // Print first few elements
        size_t print_count = std::min(static_cast<size_t>(10), data.size());
        std::cout << "  First " << print_count << " elements: [";
        for (size_t i = 0; i < print_count; ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << data[i];
        }
        std::cout << "]" << std::endl << std::endl;
    }
};

int main() {
    // Example usage: read all arrays
    for (const auto& [name, metadata] : ArraysInfo::ARRAYS) {
        std::cout << "Reading array: " << name << std::endl;
        
        // Determine data type and read accordingly
        if (metadata.dtype_code == 'f') {
            // float32
            std::vector<float> data;
            std::vector<uint32_t> shape;
            if (BinaryArrayReader::readArray(metadata.filename, data, shape)) {
                BinaryArrayReader::printArrayInfo(name, data, shape);
            }
        } else if (metadata.dtype_code == 'd') {
            // float64
            std::vector<double> data;
            std::vector<uint32_t> shape;
            if (BinaryArrayReader::readArray(metadata.filename, data, shape)) {
                BinaryArrayReader::printArrayInfo(name, data, shape);
            }
        } else if (metadata.dtype_code == 'i') {
            // int32
            std::vector<int32_t> data;
            std::vector<uint32_t> shape;
            if (BinaryArrayReader::readArray(metadata.filename, data, shape)) {
                BinaryArrayReader::printArrayInfo(name, data, shape);
            }
        } else if (metadata.dtype_code == 'l') {
            // int64
            std::vector<int64_t> data;
            std::vector<uint32_t> shape;
            if (BinaryArrayReader::readArray(metadata.filename, data, shape)) {
                BinaryArrayReader::printArrayInfo(name, data, shape);
            }
        }
    }
    
    return 0;
}
