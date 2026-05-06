#ifndef SNIPPING_H
#define SNIPPING_H

#include <easy3d/core/model.h>
#include <easy3d/core/types.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/util/logging.h>
#include <easy3d/util/resource.h>
#include <easy3d/viewer/offscreen.h>
#include <optional>
#include <string>

namespace snipping {
	struct PointsConfig {
		float point_size = 5.0f;
		easy3d::PointsDrawable::ImposterType impostor = easy3d::PointsDrawable::SPHERE;

		easy3d::State::Method coloring = easy3d::State::UNIFORM_COLOR;
		easy3d::vec4 uniform_color = easy3d::vec4(0.33f, 0.67f, 1.0f, 1.0f);

		// Visible
		bool visible = true;

		// COLOR_PROPERTY
		easy3d::State::Location color_location = easy3d::State::VERTEX;
		std::string color_property = "v:color";

		// SCALAR_FIELD
		easy3d::State::Location scalar_location = easy3d::State::VERTEX;
		std::string scalar_name;
		std::string colormap = "french";
		int num_stripes = 42;  // 0 = continuous (no discretization)
		bool clamp = false;
		float clamp_lower = 0.0f;
		float clamp_upper = 0.0f;
	};


	struct LinesConfig {
		float line_width = 3.0f;
		easy3d::LinesDrawable::ImposterType impostor = easy3d::LinesDrawable::CYLINDER;

		easy3d::State::Method coloring = easy3d::State::UNIFORM_COLOR;
		easy3d::vec3 color = easy3d::vec3(255, 175, 131) / 255.0f;
		easy3d::vec4 uniform_color = easy3d::vec4(color.x, color.y, color.z, 1.0f);

		// COLOR_PROPERTY
		easy3d::State::Location color_location = easy3d::State::VERTEX;
		std::string color_property = "e:color";

		// SCALAR_FIELD
		easy3d::State::Location scalar_location = easy3d::State::VERTEX;
		std::string scalar_name;
		std::string colormap = "default";
		int num_stripes = 0;
		bool clamp = false;
		float clamp_lower = 0.05f;
		float clamp_upper = 0.05f;
	};


	struct CameraConfig {
		std::optional<easy3d::vec3> position;
		std::optional<easy3d::quat> orientation;  // quaternion (from GUI's Ctrl+C)
		std::optional<easy3d::vec3> view_direction;
		std::optional<easy3d::vec3> up_vector;
		std::optional<easy3d::vec3> look_at;
		easy3d::Camera::Type type = easy3d::Camera::PERSPECTIVE;
		float fov = static_cast<float>(M_PI) / 4.0f;
	};


	struct OutputConfig {
		std::string file_name = "snapshot.png";
		int width = 914;
		int height = 914;
		float scaling = 1.0f;
		int samples = 0;
		int background = 2;	 // 0: current color, 1: white, 2: transparent
		easy3d::vec4 background_color = easy3d::vec4(1.0f, 1.0f, 1.0f, 1.0f);
	};


	inline void apply(easy3d::PointsDrawable* drawable, const PointsConfig& cfg) {
		if (!drawable) {
			return;
		}
		drawable->set_point_size(cfg.point_size);
		drawable->set_impostor_type(cfg.impostor);

		switch (cfg.coloring) {
			case easy3d::State::UNIFORM_COLOR:
				drawable->set_uniform_coloring(cfg.uniform_color);
				break;
			case easy3d::State::COLOR_PROPERTY:
				drawable->set_property_coloring(cfg.color_location, cfg.color_property);
				break;
			case easy3d::State::SCALAR_FIELD: {
				std::string colormap_file;
				std::vector<std::string> easy3d_colormaps = {"black_white", "blue_red", "blue_white", "blue_yellow", "default", "french", "rainbow", "random"};
				if (std::find(easy3d_colormaps.begin(), easy3d_colormaps.end(), cfg.colormap) != easy3d_colormaps.end()) {
					colormap_file = easy3d::resource::directory() + "/colormaps/" + cfg.colormap + ".png";
				} else {
					colormap_file = RESOURCE_DIR "/colormaps/" + cfg.colormap + ".png";
				}
				const easy3d::Texture* tex =
						(cfg.num_stripes > 0) ? easy3d::TextureManager::request(colormap_file, cfg.num_stripes) : easy3d::TextureManager::request(colormap_file);
				drawable->set_scalar_coloring(cfg.scalar_location, cfg.scalar_name, tex, cfg.clamp_lower, cfg.clamp_upper);
				drawable->set_clamp_range(cfg.clamp);
				break;
			}
			default:
				break;
		}
	}


	inline void apply(easy3d::LinesDrawable* drawable, const LinesConfig& cfg) {
		if (!drawable) {
			return;
		}
		drawable->set_line_width(cfg.line_width);
		drawable->set_impostor_type(cfg.impostor);

		switch (cfg.coloring) {
			case easy3d::State::UNIFORM_COLOR:
				drawable->set_uniform_coloring(cfg.uniform_color);
				break;
			case easy3d::State::COLOR_PROPERTY:
				drawable->set_property_coloring(cfg.color_location, cfg.color_property);
				break;
			case easy3d::State::SCALAR_FIELD: {
				const std::string colormap_file = easy3d::resource::directory() + "/colormaps/" + cfg.colormap + ".png";
				const easy3d::Texture* tex =
						(cfg.num_stripes > 0) ? easy3d::TextureManager::request(colormap_file, cfg.num_stripes) : easy3d::TextureManager::request(colormap_file);
				drawable->set_scalar_coloring(cfg.scalar_location, cfg.scalar_name, tex, cfg.clamp_lower, cfg.clamp_upper);
				drawable->set_clamp_range(cfg.clamp);
				break;
			}
			default:
				break;
		}
	}


	inline void apply(easy3d::Camera* camera, const CameraConfig& cfg) {
		if (!camera) {
			return;
		}
		camera->setType(cfg.type);
		camera->setFieldOfView(cfg.fov);

		if (cfg.position && cfg.orientation) {
			// GUI format: position + quaternion (from Ctrl+C in Easy3D viewer)
			camera->setPosition(*cfg.position);
			camera->setOrientation(*cfg.orientation);
		} else {
			if (cfg.up_vector) {
				camera->setUpVector(*cfg.up_vector);
			}
			if (cfg.position) {
				camera->setPosition(*cfg.position);
			}
			if (cfg.view_direction) {
				camera->setViewDirection(*cfg.view_direction);
			}
			if (cfg.look_at) {
				camera->lookAt(*cfg.look_at);
			}
		}
	}


	inline bool snapshot(easy3d::OffScreen& os, const std::string& original_ply_file, const std::string& skeleton_ply_file, const OutputConfig& output,
						 const PointsConfig& original_points_cfg = {}, const PointsConfig& skeleton_points_cfg = {},
						 const LinesConfig& lines_cfg = {}, const CameraConfig& camera_cfg = {}) {
		// Reset per-snapshot state (drop previous models and their GPU buffers).
		os.clear_scene();
		if (os.width() != output.width || os.height() != output.height) {
			os.resize(output.width, output.height);
		}
		os.set_background_color(output.background_color);

		easy3d::Model* original_model = os.add_model(original_ply_file);
		if (!original_model) {
			LOG(ERROR) << "failed to load model: " << original_ply_file;
			return false;
		}
		original_model->renderer()->set_visible(original_points_cfg.visible);
		// Configure points drawable ("vertices")
		auto* original_points = original_model->renderer()->get_points_drawable("vertices", false);
		apply(original_points, original_points_cfg);

		if (!skeleton_ply_file.empty()) {
			easy3d::Model* skeleton_model = os.add_model(skeleton_ply_file);
			if (!skeleton_model) {
				LOG(ERROR) << "failed to load model: " << skeleton_ply_file;
				return false;
			}
			// Configure points drawable ("vertices")
			auto* skeleton_points = skeleton_model->renderer()->get_points_drawable("vertices", false);
			apply(skeleton_points, skeleton_points_cfg);
			// Configure lines drawable ("edges", present in Graph models)
			auto* skeleton_edges = skeleton_model->renderer()->get_lines_drawable("edges", false);
			apply(skeleton_edges, lines_cfg);
		}

		// Camera setup
		easy3d::Camera* cam = os.camera();
		const auto& bbox = original_model->bounding_box();
		cam->setSceneBoundingBox(bbox.min_point(), bbox.max_point());
		cam->showEntireScene();
		apply(cam, camera_cfg);

		return os.render(output.file_name, output.scaling, output.samples, output.background);
	}
}  // namespace snipping

#endif	// SNIPPING_H
