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

#include <hatchbed_common/pointcloud/self_shapes.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <resource_retriever/retriever.hpp>
#include <urdf/model.h>

namespace hatchbed_common {
namespace pointcloud {

namespace {

// Facets per turn for a cylinder or a sphere. The polygon is circumscribed rather than
// inscribed, so a facet never cuts inside the surface it stands for and the prism never
// under-covers; sixteen keeps the excess at a corner under 2% of the radius.
constexpr int kFacets = 16;

// Of the padding, how much of it a cached prism is allowed to have drifted out of true by
// before it is rebuilt from vertices. Half, so the shortcut is always well inside the margin
// the padding was there to provide in the first place.
constexpr float kTiltBudget = 0.5f;

Eigen::Isometry3f toIsometry(const urdf::Pose& pose) {
    Eigen::Isometry3f out = Eigen::Isometry3f::Identity();
    out.translation() = Eigen::Vector3f(static_cast<float>(pose.position.x),
                                        static_cast<float>(pose.position.y),
                                        static_cast<float>(pose.position.z));
    const Eigen::Quaternionf rotation(
        static_cast<float>(pose.rotation.w), static_cast<float>(pose.rotation.x),
        static_cast<float>(pose.rotation.y), static_cast<float>(pose.rotation.z));
    if (rotation.norm() > 1e-6f) {
        out.linear() = rotation.normalized().toRotationMatrix();
    }
    return out;
}

void addBox(const urdf::Box& box, std::vector<Eigen::Vector3f>& out) {
    const Eigen::Vector3f half(static_cast<float>(box.dim.x) * 0.5f,
                               static_cast<float>(box.dim.y) * 0.5f,
                               static_cast<float>(box.dim.z) * 0.5f);
    for (int i = 0; i < 8; ++i) {
        out.emplace_back((i & 1) ? half.x() : -half.x(), (i & 2) ? half.y() : -half.y(),
                         (i & 4) ? half.z() : -half.z());
    }
}

// A circumscribed ring at each end. Radius is grown by 1 / cos(pi / facets) so the flats sit
// outside the true circle rather than chording across it.
void addRing(float radius, float z, std::vector<Eigen::Vector3f>& out) {
    const float outer = radius / std::cos(static_cast<float>(M_PI) / kFacets);
    for (int i = 0; i < kFacets; ++i) {
        const float angle = 2.0f * static_cast<float>(M_PI) * i / kFacets;
        out.emplace_back(outer * std::cos(angle), outer * std::sin(angle), z);
    }
}

void addCylinder(const urdf::Cylinder& cylinder, std::vector<Eigen::Vector3f>& out) {
    const float half_length = static_cast<float>(cylinder.length) * 0.5f;
    addRing(static_cast<float>(cylinder.radius), -half_length, out);
    addRing(static_cast<float>(cylinder.radius), half_length, out);
}

// A sphere's prism is its equatorial circle over its full height, so a ring at the equator
// gives the footprint and the poles give the Z extent.
void addSphere(const urdf::Sphere& sphere, std::vector<Eigen::Vector3f>& out) {
    const float radius = static_cast<float>(sphere.radius);
    addRing(radius, 0.0f, out);
    out.emplace_back(0.0f, 0.0f, -radius);
    out.emplace_back(0.0f, 0.0f, radius);
}

// Walks an Assimp scene, applying each node's transform, since a glTF's geometry means
// nothing without the node hierarchy that places it.
void addNode(const aiScene& scene, const aiNode& node, const Eigen::Matrix4f& parent,
             const Eigen::Vector3f& scale, std::vector<Eigen::Vector3f>& out) {
    Eigen::Matrix4f local;
    const aiMatrix4x4& m = node.mTransformation;
    local << m.a1, m.a2, m.a3, m.a4,
             m.b1, m.b2, m.b3, m.b4,
             m.c1, m.c2, m.c3, m.c4,
             m.d1, m.d2, m.d3, m.d4;
    const Eigen::Matrix4f world = parent * local;

    for (unsigned int i = 0; i < node.mNumMeshes; ++i) {
        const aiMesh* mesh = scene.mMeshes[node.mMeshes[i]];
        if (mesh == nullptr) {
            continue;
        }
        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            const aiVector3D& p = mesh->mVertices[v];
            const Eigen::Vector4f placed = world * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
            out.emplace_back(placed.x() * scale.x(), placed.y() * scale.y(),
                             placed.z() * scale.z());
        }
    }
    for (unsigned int i = 0; i < node.mNumChildren; ++i) {
        addNode(scene, *node.mChildren[i], world, scale, out);
    }
}

// Every vertex of a mesh, not its convex hull: a hull in three dimensions is a dependency
// this does not have, and the vertices are only ever fed to a two-dimensional hull anyway.
bool addMesh(const urdf::Mesh& mesh, const std::function<void(const std::string&)>& warn,
             std::vector<Eigen::Vector3f>& out) {
    resource_retriever::MemoryResource resource;
    try {
        resource_retriever::Retriever retriever;
        resource = retriever.get(mesh.filename);
    } catch (const std::exception& error) {
        warn("cannot retrieve '" + mesh.filename + "': " + error.what());
        return false;
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFileFromMemory(
        resource.data.get(), resource.size, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices,
        mesh.filename.c_str());
    if (scene == nullptr || scene->mRootNode == nullptr) {
        warn("cannot read '" + mesh.filename + "': " + importer.GetErrorString());
        return false;
    }

    const Eigen::Vector3f scale(static_cast<float>(mesh.scale.x),
                                static_cast<float>(mesh.scale.y),
                                static_cast<float>(mesh.scale.z));
    const size_t before = out.size();
    addNode(*scene, *scene->mRootNode, Eigen::Matrix4f::Identity(), scale, out);
    if (out.size() == before) {
        warn("'" + mesh.filename + "' holds no vertices");
        return false;
    }
    return true;
}

// One element's geometry, in its link's frame.
std::vector<Eigen::Vector3f> verticesOf(const urdf::Geometry& geometry, const urdf::Pose& origin,
                                        const std::function<void(const std::string&)>& warn) {
    std::vector<Eigen::Vector3f> local;
    switch (geometry.type) {
        case urdf::Geometry::BOX:
            addBox(static_cast<const urdf::Box&>(geometry), local);
            break;
        case urdf::Geometry::CYLINDER:
            addCylinder(static_cast<const urdf::Cylinder&>(geometry), local);
            break;
        case urdf::Geometry::SPHERE:
            addSphere(static_cast<const urdf::Sphere&>(geometry), local);
            break;
        case urdf::Geometry::MESH:
            if (!addMesh(static_cast<const urdf::Mesh&>(geometry), warn, local)) {
                return {};
            }
            break;
        default:
            warn("unsupported geometry type");
            return {};
    }

    const Eigen::Isometry3f pose = toIsometry(origin);
    for (auto& vertex : local) {
        vertex = pose * vertex;
    }
    return local;
}

// A link's geometry as one vertex list per element, from whichever of its collision and
// visual elements were asked for. Empty if the link carries none of them, which is most of a
// URDF.
std::vector<std::vector<Eigen::Vector3f>> elementsOf(
    const urdf::Link& link, SelfShapeConfig::Geometry geometry,
    const std::function<void(const std::string&)>& warn) {
    std::vector<std::vector<Eigen::Vector3f>> elements;

    const bool want_collision = geometry != SelfShapeConfig::Geometry::Visual;
    // Collision alone falls back to the visuals when there are none, so that a link the
    // model never gave a collision shape is still covered.
    const bool want_visual = geometry != SelfShapeConfig::Geometry::Collision ||
                             link.collision_array.empty();

    if (want_collision) {
        for (const auto& collision : link.collision_array) {
            if (!collision || !collision->geometry) {
                continue;
            }
            auto vertices = verticesOf(*collision->geometry, collision->origin, warn);
            if (!vertices.empty()) {
                elements.push_back(std::move(vertices));
            }
        }
    }
    if (want_visual) {
        for (const auto& visual : link.visual_array) {
            if (!visual || !visual->geometry) {
                continue;
            }
            auto vertices = verticesOf(*visual->geometry, visual->origin, warn);
            if (!vertices.empty()) {
                elements.push_back(std::move(vertices));
            }
        }
    }
    return elements;
}

}  // namespace

SelfShape::SelfShape(std::string name, std::vector<Source> sources, float padding)
: name_(std::move(name)), sources_(std::move(sources)), padding_(padding) {}

void SelfShape::rebuild(const std::vector<Eigen::Isometry3f>& poses) {
    std::vector<Eigen::Vector3f> placed;
    size_t count = 0;
    for (const auto& source : sources_) {
        count += source.vertices.size();
    }
    placed.reserve(count);

    for (size_t i = 0; i < sources_.size(); ++i) {
        for (const auto& vertex : sources_[i].vertices) {
            placed.push_back(poses[i] * vertex);
        }
    }

    prism_ = padConvexPrism(makeConvexPrism(placed), padding_);
    bounds_ = boundsOfConvexPrism(prism_);
    built_at_ = poses;
    built_ = !prism_.hull.empty();
}

bool SelfShape::update(
    const std::function<std::optional<Eigen::Isometry3f>(const std::string&)>& lookup) {
    std::vector<Eigen::Isometry3f> poses;
    poses.reserve(sources_.size());
    for (const auto& source : sources_) {
        const std::optional<Eigen::Isometry3f> pose = lookup(source.frame);
        if (!pose) {
            return false;
        }
        poses.push_back(*pose);
    }
    if (poses.empty()) {
        return false;
    }

    // More than one source means the members can move relative to each other, so their shared
    // hull only ever described one configuration and there is nothing to carry across.
    if (!built_ || sources_.size() != 1) {
        rebuild(poses);
        return built_;
    }

    const Eigen::Isometry3f delta = poses.front() * built_at_.front().inverse();
    // How far the top of the prism moves out of line with its bottom under this delta. Zero
    // for a translation or a turn about Z, which is all a body-frame filter usually sees.
    const Eigen::Vector3f up = delta.linear() * Eigen::Vector3f::UnitZ();
    const float tilt_sine = up.head<2>().norm();
    const float height = std::max(0.0f, prism_.max_z - prism_.min_z);
    if (height * tilt_sine > kTiltBudget * padding_) {
        rebuild(poses);
        return built_;
    }

    prism_ = transformConvexPrism(prism_, delta);
    bounds_ = boundsOfConvexPrism(prism_);
    built_at_ = poses;
    return true;
}

std::vector<SelfShape> buildSelfShapes(const std::string& urdf, const SelfShapeConfig& config,
                                        const std::function<void(const std::string&)>& warn) {
    urdf::Model model;
    if (!model.initString(urdf)) {
        warn("the robot description will not parse");
        return {};
    }

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);

    // Read once here rather than at every use: a link's geometry does not depend on how it
    // is going to be grouped, and a mesh should not be loaded twice because two groups name
    // the same link.
    std::map<std::string, std::vector<std::vector<Eigen::Vector3f>>> elements;
    for (const auto& link : links) {
        if (!link || config.disabled_links.count(link->name) > 0) {
            continue;
        }
        const auto override_geometry = config.link_geometry.find(link->name);
        const SelfShapeConfig::Geometry geometry =
            override_geometry == config.link_geometry.end() ? config.geometry
                                                            : override_geometry->second;
        auto found = elementsOf(*link, geometry, [&](const std::string& what) {
            warn("link '" + link->name + "': " + what);
        });
        if (!found.empty()) {
            elements[link->name] = std::move(found);
        }
    }

    const auto paddingFor = [&](const std::string& link) {
        const auto found = config.link_padding.find(link);
        return found == config.link_padding.end() ? config.padding : found->second;
    };

    std::vector<SelfShape> shapes;
    std::set<std::string> grouped;

    // Groups first: a link in one is not evaluated on its own, and a group is always exactly
    // one prism -- combining is the whole reason to ask for one.
    for (const auto& group : config.groups) {
        std::vector<SelfShape::Source> sources;
        float padding = config.padding;
        size_t absent = 0;
        for (const auto& link : group.links) {
            grouped.insert(link);
            // A link this description does not have at all is a different thing from a link
            // it has and cannot contribute: one configuration usually has to serve every
            // variant of a machine, and naming the other variant's links is how it does. So
            // that is counted and reported once below, while a link that IS here and offers
            // nothing is a modelling gap and says so per link.
            if (!model.getLink(link)) {
                ++absent;
                continue;
            }
            const auto found = elements.find(link);
            if (found == elements.end()) {
                warn("group '" + group.name + "' names '" + link +
                     "', which is in this description but has no geometry to contribute");
                continue;
            }
            std::vector<Eigen::Vector3f> all;
            for (const auto& element : found->second) {
                all.insert(all.end(), element.begin(), element.end());
            }
            sources.push_back({link, std::move(all)});
            padding = std::max(padding, paddingFor(link));
        }
        if (sources.empty()) {
            if (absent == group.links.size()) {
                warn("group '" + group.name + "' describes structure this robot does not "
                     "have, so it is skipped");
            } else {
                warn("group '" + group.name + "' has no geometry at all and is dropped");
            }
            continue;
        }
        if (absent > 0) {
            warn("group '" + group.name + "' names " + std::to_string(absent) +
                 " link(s) this description does not have, and is built from the rest");
        }
        shapes.emplace_back("group:" + group.name, std::move(sources), padding);
    }

    for (const auto& [link, link_elements] : elements) {
        if (grouped.count(link) > 0) {
            continue;
        }
        const auto override_combine = config.link_combine.find(link);
        const bool combine = override_combine == config.link_combine.end()
                                 ? config.combine_elements
                                 : override_combine->second;
        const float padding = paddingFor(link);

        if (combine) {
            std::vector<Eigen::Vector3f> all;
            for (const auto& element : link_elements) {
                all.insert(all.end(), element.begin(), element.end());
            }
            shapes.emplace_back(link, std::vector<SelfShape::Source>{{link, std::move(all)}},
                                padding);
            continue;
        }

        for (size_t i = 0; i < link_elements.size(); ++i) {
            shapes.emplace_back(link + "[" + std::to_string(i) + "]",
                                std::vector<SelfShape::Source>{{link, link_elements[i]}},
                                padding);
        }
    }

    return shapes;
}

}  // namespace pointcloud
}  // namespace hatchbed_common
