#include "../include/sdk_core.hpp"

namespace physics {
namespace sdk {

// Version information function to make sure we have at least one symbol
// in our library to link against
extern "C" {
    const char* getSdkVersion() {
        return "Physics Engine SDK v1.0";
    }
}

} // namespace sdk
} // namespace physics
