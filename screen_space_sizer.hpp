#ifndef SCREEN_SPACE_SIZER_HPP
#define SCREEN_SPACE_SIZER_HPP

#include "sbpt_generated_includes.hpp"
#include <algorithm>

class ScreenSpaceSizer {
  public:
    enum class Size { Large, Medium, Small };

    explicit ScreenSpaceSizer(const ICamera &cam, const unsigned int &screen_width_px,
                              const unsigned int &screen_height_px)
        : camera(cam), screen_width_px(screen_width_px), screen_height_px(screen_height_px) {}

    Size get_screen_size(std::vector<glm::vec3> xyz_positions, Transform transform) const {

        auto local_aabb = vertex_geometry::AxisAlignedBoundingBox(xyz_positions);
        float pixel_area = compute_screen_pixel_area(local_aabb, transform);
        AABB2D pixel_bounding_box = compute_pixel_bounding_box(local_aabb, transform);

        float min_pixel_dimension = pixel_bounding_box.min_dimension();

        if (min_pixel_dimension > 20.0f)
            return Size::Large;

        if (min_pixel_dimension > 10.0f)
            return Size::Medium;

        return Size::Small;
    }

    template <draw_info::IVPLike IVPX> draw_info::IndexedVertexPositions make_screen_space_ivp(IVPX &obj) const {
        if (obj.xyz_positions.empty()) {
            return {}; // empty object
        }

        auto box = vertex_geometry::AxisAlignedBoundingBox(obj.xyz_positions);

        auto corners_world = get_aabb_corners_world(box, obj.transform);

        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        float aspect = static_cast<float>(screen_width_px) / static_cast<float>(screen_height_px);

        // --- Project to NDC and correct for aspect ratio ---
        for (const auto &corner : corners_world) {
            glm::vec2 ndc = project_to_ndc(corner);
            ndc.x *= aspect; // correct horizontal scaling

            if (!std::isfinite(ndc.x) || !std::isfinite(ndc.y)) {
                continue; // skip invalid projections
            }

            min_x = std::min(min_x, ndc.x);
            max_x = std::max(max_x, ndc.x);
            min_y = std::min(min_y, ndc.y);
            max_y = std::max(max_y, ndc.y);
        }

        min_x = std::max(min_x, -aspect); // clamp using aspect-corrected limits
        max_x = std::min(max_x, aspect);
        min_y = std::max(min_y, -1.0f);
        max_y = std::min(max_y, 1.0f);

        // --- Build IVP in NDC ---
        draw_info::IndexedVertexPositions ivp;
        ivp.xyz_positions = {
            glm::vec3(min_x, min_y, 0.0f), // bottom-left
            glm::vec3(max_x, min_y, 0.0f), // bottom-right
            glm::vec3(max_x, max_y, 0.0f), // top-right
            glm::vec3(min_x, max_y, 0.0f)  // top-left
        };

        ivp.indices = {0, 1, 2, 2, 3, 0};
        ivp.transform = Transform{};

        return ivp;
    }

  private:
    const ICamera &camera;
    const unsigned int &screen_width_px, &screen_height_px;

    // TODO: don't need this function just need a function that takes in a mat and and a vector of vec3s and applies to
    // all of them.
    std::array<glm::vec3, 8> get_aabb_corners_world(const vertex_geometry::AxisAlignedBoundingBox &box,
                                                    Transform &transform) const {
        glm::mat4 model = transform.get_transform_matrix();
        std::array<glm::vec3, 8> corners = box.get_corners();
        for (auto &c : corners) {
            glm::vec4 world = model * glm::vec4(c, 1.0f);
            c = glm::vec3(world);
        }
        return corners;
    }

    // TODO: move to vertex geom eventually.
    glm::vec2 project_to_ndc(const glm::vec3 &world_pos) const {
        glm::mat4 view = camera.get_view_matrix();
        glm::mat4 proj = camera.get_projection_matrix();

        glm::vec4 clip = proj * view * glm::vec4(world_pos, 1.0f);
        if (clip.w == 0.0f)
            return glm::vec2(0.0f, 0.0f); // avoid divide by zero

        glm::vec3 ndc = glm::vec3(clip) / clip.w; // perspective divide

        return glm::vec2(ndc.x, ndc.y); // NDC coordinates in [-1, 1]
    }

    glm::vec2 project_to_screen(const glm::vec3 &world_pos) const {
        glm::mat4 view = camera.get_view_matrix();
        glm::mat4 proj = camera.get_projection_matrix();

        glm::vec4 clip = proj * view * glm::vec4(world_pos, 1.0f);
        glm::vec3 ndc = glm::vec3(clip) / clip.w;

        glm::vec2 screen;
        screen.x = (ndc.x * 0.5f + 0.5f) * screen_width_px;
        screen.y = (1.0f - (ndc.y * 0.5f + 0.5f)) * screen_height_px;
        return screen;
    }

    struct AABB2D {
        glm::vec2 min;
        glm::vec2 max;

        float width() const { return std::max(0.0f, max.x - min.x); }

        float height() const { return std::max(0.0f, max.y - min.y); }

        float area() const { return width() * height(); }

        float min_dimension() const { return std::min(width(), height()); }

        float max_dimension() const { return std::max(width(), height()); }
    };

    AABB2D compute_pixel_bounding_box(const vertex_geometry::AxisAlignedBoundingBox &box, Transform &transform) const {
        auto corners = get_aabb_corners_world(box, transform);

        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (size_t i = 0; i < corners.size(); ++i) {
            glm::vec2 screen = project_to_screen(corners[i]);

            // clamp to viewport bounds
            screen.x = std::clamp(screen.x, 0.0f, static_cast<float>(screen_width_px));
            screen.y = std::clamp(screen.y, 0.0f, static_cast<float>(screen_height_px));

            min_x = std::min(min_x, screen.x);
            max_x = std::max(max_x, screen.x);
            min_y = std::min(min_y, screen.y);
            max_y = std::max(max_y, screen.y);
        }

        return {{min_x, min_y}, {max_x, max_y}};
    }

    float compute_screen_pixel_area(const vertex_geometry::AxisAlignedBoundingBox &box, Transform &transform) const {
        AABB2D bb = compute_pixel_bounding_box(box, transform);

        float width = std::max(0.0f, bb.max.x - bb.min.x);
        float height = std::max(0.0f, bb.max.y - bb.min.y);
        return width * height;
    }

    float compute_screen_pixel_area_percentage(const vertex_geometry::AxisAlignedBoundingBox &box,
                                               Transform &transform) {
        float screen_area = static_cast<float>(screen_width_px) * static_cast<float>(screen_height_px);
        float percentage = (compute_screen_pixel_area(box, transform) / screen_area) * 100.0f;
        return percentage;
    }
};

#endif // SCREEN_SPACE_SIZER_HPP
