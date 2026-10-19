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

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <hatchbed_common/pointcloud/convex_prism.hpp>

namespace hatchbed_common {
namespace pointcloud {

// The robot's own geometry, as convex prisms in a body frame, for deciding which of a
// sensor's returns are the robot looking at itself.
//
// ## Where the geometry comes from
//
// A URDF, per link, from its <collision> elements, its <visual> ones, or both.
//
// Both by default, because the two are approximations of the same structure and a filter's
// failure mode is missing coverage. Collision geometry is often deliberately cruder than the
// visual it stands beside -- a chassis whose collision is two boxes and whose visual is the
// CAD frame leaves out the battery box and the bumper; a deck slab as one box leaves out
// everything hanging under it; an arm as one box leaves out the bar bolted across it. Taking
// the union of the two can only over-cover, and over-covering the machine's own volume costs
// nothing that under-covering does not cost twice.
//
// Collision alone is the cheaper choice and the right one where collision geometry is an
// inflated superset of the visual rather than a simplification of it. Visual alone suits a
// model whose collision shapes were built for contacts and are nowhere near the surface.
//
// ## Why prisms
//
// A prism is a footprint extruded along Z. For a wheeled machine that is not the
// approximation it sounds: chassis boxes, deck slabs, guardrail posts, sensor bodies and
// mast sections are all prismatic about the vehicle's own Z. It over-covers only members
// that lean -- lift arms, struts -- and those sit inside the envelope of the things around
// them. What it buys is the test: a Z range and a convex polygon, no square roots.
//
// ## Granularity
//
// Every element of every link as its own prism is the most faithful and the most expensive.
// Config trades that off in two ways, and both exist because the right answer is a property
// of the machine rather than of the code:
//
//   combine   -- one prism for a whole link instead of one per element.
//   groups    -- one prism for several links at once, a "virtual link".
//
// Combining fills whatever the elements did not: combining a guardrail's posts fills the
// cage, which is fine until something the sensor SHOULD see is inside it. So the useful
// combinations are the ones whose elements are already adjacent -- a chassis in two boxes,
// a lift arm as a body and two pins.
struct SelfShapeConfig {
    // Which of a link's geometry to build from. See the class comment on why the union is
    // the default.
    enum class Geometry {
        // Collision elements if the link has any, otherwise its visuals.
        Collision,
        Visual,
        Both,
    };
    Geometry geometry = Geometry::Both;
    // Per-link override of the above.
    std::map<std::string, Geometry> link_geometry;

    // One prism per link rather than one per element, where a link does not say otherwise.
    bool combine_elements = false;
    // Per-link override of the above.
    std::map<std::string, bool> link_combine;
    // Links to leave out entirely. The cheapest granularity control there is.
    std::set<std::string> disabled_links;
    // How far every prism is grown, in metres, to cover the difference between a model and
    // a machine. Per-link overrides sit alongside.
    float padding = 0.05f;
    std::map<std::string, float> link_padding;

    // A virtual link: one prism built from several real links at once. A link named here is
    // no longer evaluated on its own. Naming one in two groups is allowed and simply means
    // its geometry lands in both.
    struct Group {
        std::string name;
        std::vector<std::string> links;
    };
    std::vector<Group> groups;
};

// One prism the filter tests against, and what it was built from.
class SelfShape {
 public:
    // Geometry from one link, in that link's own frame.
    struct Source {
        std::string frame;
        // Vertices whose convex hull covers the geometry: a box's corners, a circumscribed
        // polygon for a cylinder or sphere, a mesh's own vertices. The element's origin and
        // any mesh scale are already in them. Kept rather than discarded after the first
        // prism is built, because a prism cannot be tilted without going back to these --
        // see transformConvexPrism.
        std::vector<Eigen::Vector3f> vertices;
    };

    SelfShape(std::string name, std::vector<Source> sources, float padding);

    const std::string& name() const { return name_; }
    const std::vector<Source>& sources() const { return sources_; }
    // Valid only after update() has returned true at least once.
    const ConvexPrism& prism() const { return prism_; }
    const PrismBounds& bounds() const { return bounds_; }

    /**
     * Brings the prism up to date for where the robot is now.
     *
     * Rebuilding from the vertices is always correct and is what happens whenever the shape
     * has tilted since it was last built, or has more than one source (whose members can
     * move relative to each other, so their common hull is only ever valid for one
     * configuration). Otherwise the cached prism is carried across by transform, which is
     * exact for the translation and yaw that a body-frame filter sees almost all of.
     *
     * "Has tilted" is measured against the padding rather than against an angle: a tilt of
     * t displaces the top of a prism of height h by h * sin(t), so the transform is taken
     * while that displacement stays under half the padding already carried, and the shape is
     * rebuilt once it does not. That makes the cheap path provably inside the margin instead
     * of merely usually inside it.
     *
     * @param[in] lookup  Returns the pose of a source's frame in the frame the prisms are
     *                    held in, or nothing if it is not available -- in which case this
     *                    shape is left as it was and false is returned.
     *
     * @return True if the shape now describes the pose lookup reports.
     */
    bool update(const std::function<std::optional<Eigen::Isometry3f>(const std::string&)>&
                    lookup);

 private:
    void rebuild(const std::vector<Eigen::Isometry3f>& poses);

    std::string name_;
    std::vector<Source> sources_;
    float padding_ = 0.0f;

    ConvexPrism prism_;
    PrismBounds bounds_;
    // The poses the cached prism was built at, one per source, and whether it holds anything.
    std::vector<Eigen::Isometry3f> built_at_;
    bool built_ = false;
};

/**
 * Reads a URDF into the shapes a self filter tests against.
 *
 * @param[in] urdf    The robot description, as XML.
 * @param[in] config  What to build and how finely. See SelfShapeConfig.
 * @param[in] warn    Called with anything that was skipped and why -- a mesh that would not
 *                    load, a group naming a link that does not exist. Skipping is reported
 *                    rather than fatal: a filter missing one shape is worth having, and the
 *                    node that owns the logger decides how loudly to say so.
 *
 * @return The shapes, in no particular order. Empty if the URDF will not parse.
 */
std::vector<SelfShape> buildSelfShapes(const std::string& urdf, const SelfShapeConfig& config,
                                        const std::function<void(const std::string&)>& warn);

}  // namespace pointcloud
}  // namespace hatchbed_common
