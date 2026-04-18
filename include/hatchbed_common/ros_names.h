// Copyright 2026 Hatchbed L.L.C.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cctype>
#include <string>

namespace hatchbed_common {

/**
 * Returns true when name is a valid ROS 2 parameter name component.
 *
 * Rules (matching those of rmw_validate_node_name):
 *   - must not be empty
 *   - must not exceed 255 characters
 *   - first character must be a letter (a-z, A-Z) or underscore
 *   - remaining characters must be alphanumeric (a-z, A-Z, 0-9) or underscore
 *
 * These are the same constraints enforced on ROS 2 node names and are
 * suitable for any string used as a single component of a dotted parameter
 * name (e.g., the prefix before a '.' separator).
 */
inline bool isValidParamName(const std::string& name) {
    if (name.empty() || name.size() > 255) { return false; }
    const auto first = static_cast<unsigned char>(name[0]);
    if (!std::isalpha(first) && name[0] != '_') { return false; }
    for (size_t i = 1; i < name.size(); ++i) {
        const auto c = static_cast<unsigned char>(name[i]);
        if (!std::isalnum(c) && name[i] != '_') { return false; }
    }
    return true;
}

}  // namespace hatchbed_common
