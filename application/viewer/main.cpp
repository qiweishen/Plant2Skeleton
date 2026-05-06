#include "viewer.h"

// Easy3D
#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/renderer/drawable.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/util/dialog.h>
#include <easy3d/util/initializer.h>

// ImGui
#include <3rd_party/imgui/backends/imgui_impl_opengl3.h>
#include <3rd_party/imgui/imgui.h>

// SkelSeg
#include <nlohmann/json.hpp>

#include "graph.h"
#include "lop.h"
#include "skeleton.h"
#include "tools.h"
#include "utility/config_validator.h"
#include "utility/logger.h"

// Standard Library
#include <atomic>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <regex>
#include <thread>

#include "easy3d/renderer/camera.h"



class GuiLogBuffer : public std::streambuf {
public:
	void install(std::ostream &stream) {
		original_ = stream.rdbuf(this);
		stream_ = &stream;
	}

	void uninstall() {
		if (stream_ && original_) {
			stream_->rdbuf(original_);
			original_ = nullptr;
			stream_ = nullptr;
		}
	}

	void add(const std::string &msg) {
		std::lock_guard<std::mutex> lock(mutex_);
		lines_.push_back(msg);
		if (lines_.size() > kMaxLines) {
			lines_.pop_front();
		}
		scroll_ = true;
	}

	void clear() {
		std::lock_guard<std::mutex> lock(mutex_);
		lines_.clear();
	}

	// Render under lock and consume scroll flag
	template<typename F>
	bool render(F &&f) {
		std::lock_guard<std::mutex> lock(mutex_);
		for (const auto &l: lines_) {
			f(l);
		}
		bool s = scroll_;
		scroll_ = false;
		return s;
	}

protected:
	int overflow(int c) override {
		std::lock_guard<std::mutex> lock(mutex_);
		if (c == '\n') {
			lines_.push_back(cur_);
			if (lines_.size() > kMaxLines) {
				lines_.pop_front();
			}
			cur_.clear();
			scroll_ = true;
		} else if (c != EOF) {
			cur_ += static_cast<char>(c);
		}
		if (original_) {
			original_->sputc(c);
		}
		return c;
	}

	std::streamsize xsputn(const char *s, std::streamsize n) override {
		for (std::streamsize i = 0; i < n; ++i)
			overflow(s[i]);
		return n;
	}

private:
	std::streambuf *original_ = nullptr;
	std::ostream *stream_ = nullptr;
	std::deque<std::string> lines_;
	std::string cur_;
	std::mutex mutex_;
	bool scroll_ = false;
	static constexpr size_t kMaxLines = 2000;
};

static GuiLogBuffer g_log;


// Pipeline stage identifiers
enum class Stage : int {
	NONE = 0,
	LOAD_POINT_CLOUD = 1,
	PREPROCESS = 2,
	LAPLACIAN = 3,
	SKEL_DOWNSAMPLE = 4,
	LOP_CALIBRATION = 5,
	BUILD_GRAPH = 6,
	COMPUTE_MST = 7,
	PRUNE_MST = 8,
	SEGMENT_SKELETON = 9,
	ASSIGN_LABELS = 10
};

static const char *kStageNames[] = {
	"None",			  "1. Load Point Cloud", "2. Preprocess", "3. Laplacian Contraction", "4. Skeleton Downsample", "5. LOP Calibration",
	"6. Build Graph", "7. Compute MST",		 "8. Prune MST",  "9. Segment Skeleton",	  "10. Assign Labels"
};

static constexpr int kStageCount = 10;


// Help function
std::filesystem::path find_latest_cpts(const std::filesystem::path &dir) {
	static const std::regex pattern(R"(cpts_(\d+)\.ply)");

	std::filesystem::path best_path;
	long long best_num = -1;

	for (const auto &entry: std::filesystem::directory_iterator(dir)) {
		if (!entry.is_regular_file()) {
			continue;
		}

		const std::string fname = entry.path().filename().string();
		std::smatch m;
		if (!std::regex_match(fname, m, pattern)) {
			continue;
		}

		const long long num = std::stoll(m[1].str());
		if (num > best_num) {
			best_num = num;
			best_path = entry.path();
		}
	}

	if (best_num < 0) {
		throw std::runtime_error("No cpts_N.ply in " + dir.string());
	}
	return best_path;
}


std::filesystem::path find_result_ply(const std::filesystem::path &dir) {
	std::filesystem::path found;
	for (const auto &entry: std::filesystem::directory_iterator(dir)) {
		if (!entry.is_regular_file()) {
			continue;
		}

		const std::string fname = entry.path().filename().string();
		if (!fname.ends_with("_Result.ply")) {
			continue;
		}

		if (!found.empty()) {
			throw std::runtime_error("Multiple *_Result.ply files in " + dir.string());
		}
		found = entry.path();
	}

	if (found.empty()) {
		throw std::runtime_error("No *_Result.ply found in " + dir.string());
	}
	return found;
}


// MyViewer
class MyViewer : public easy3d::ViewerImGui {
public:
	MyViewer() : ViewerImGui("SkelSeg", 4, 3, 2, false, true, 24, 8, 1600, 900) {
		g_log.install(std::cout);
		easy3d_usage_ = manual_;
		manual_ = "";
	}

	~MyViewer() override {
		if (worker_.joinable()) {
			worker_.join();
		}
		g_log.uninstall();
	}

protected:
	// post_draw (replaces ViewerImGui version)
	void post_draw() override {
		show_easy3d_logo_ = false;

		drain_pending_visualizations();

		// ---- Floating panels ----
		if (show_about_) {
			draw_about();
		}
		if (show_help_) {
			draw_help();
		}
		if (show_param_panel_) {
			draw_parameter_panel();
		}
		if (show_pipeline_panel_) {
			draw_pipeline_panel();
		}
		if (show_log_panel_) {
			draw_log_panel();
		}
		if (show_layer_manager_) {
			draw_layer_manager();
		}

		// ---- Modal dialogs ----
		draw_clear_output_popup();

		// ---- Main menu bar ----
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
		if (ImGui::BeginMainMenuBar()) {
			draw_menu_file();

			if (ImGui::BeginMenu("View")) {
				if (ImGui::MenuItem("Snapshot", nullptr)) {
					snapshot();
				}
				ImGui::Separator();
				ImGui::MenuItem("Layer Manager", nullptr, &show_layer_manager_);
				ImGui::MenuItem("Log", nullptr, &show_log_panel_);
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Parameters Setting")) {
				if (ImGui::MenuItem("Load from File", nullptr)) {
					load_parameter_file();
				}
				ImGui::Separator();
				ImGui::MenuItem("Parameter Panel", nullptr, &show_param_panel_);
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Skeleton Extraction")) {
				if (ImGui::MenuItem("Run All", nullptr, false, cfg_ok_ && !is_running_ && !cfg_dirty_)) {
					run_all();
				}
				if (cfg_dirty_ && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
					ImGui::SetTooltip("Click 'Confirm Parameters' first.");
				}
				ImGui::Separator();
				ImGui::MenuItem("Pipeline Control", nullptr, &show_pipeline_panel_);
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Help")) {
				ImGui::MenuItem("About", nullptr, &show_about_);
				ImGui::Separator();
				ImGui::MenuItem("Help", nullptr, &show_help_);
				ImGui::EndMenu();
			}

			menu_height_ = ImGui::GetWindowHeight();
			ImGui::EndMainMenuBar();
		}
		ImGui::PopStyleVar();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		Viewer::post_draw();
	}

private:
	// Easy3D viewer shortcut
	std::string easy3d_usage_;

	// UI toggle flags
	bool show_about_ = false;
	bool show_help_ = false;
	bool show_param_panel_ = true;
	bool show_pipeline_panel_ = true;
	bool show_log_panel_ = true;
	bool show_layer_manager_ = false;

	// Configuration state
	nlohmann::json cfg_;
	std::filesystem::path cfg_path_;
	std::string base_output_path_;	// base output dir (no stem appended); final = base / stem(pc)
	bool cfg_ok_ = false;
	bool cfg_dirty_ = false;		// staged but not yet validated via Confirm Parameters; gates Run/exec buttons
	bool pending_output_has_contents_ = false;
	std::size_t pending_output_item_count_ = 0;
	std::vector<std::string> pending_output_preview_;
	std::filesystem::path pending_output_path_;
	std::function<void()> pending_after_confirm_;
	std::function<void()> pending_after_cancel_;
	bool show_clear_output_popup_ = false;
	bool dont_ask_again_session_ = false;

	char buf_pc_path_[512] = {};
	char buf_sem_name_[128] = {};
	char buf_inst_name_[128] = {};
	char buf_out_dir_[512] = {};

	// Pipeline state
	std::atomic<int> completed_{ 0 };
	std::atomic<int> running_stage_{ 0 };
	std::atomic<bool> is_running_{ false };
	std::string error_msg_;
	mutable std::mutex error_mu_;
	std::mutex vis_mu_;
	std::vector<Stage> vis_pending_;
	std::thread worker_;

	// Pipeline intermediate data
	Eigen::MatrixXd original_cloud_;
	Eigen::MatrixXd input_cloud_;
	Eigen::MatrixXd lap_skeleton_;
	Eigen::MatrixXd fps_points_;
	Eigen::MatrixXd lop_points_;
	std::unique_ptr<::Graph> graph_obj_;
	Boost_Graph initial_graph_;
	Boost_Graph mst_graph_;
	Boost_Graph pruned_mst_;
	std::vector<int> skel_sem_;
	std::vector<int> skel_inst_;
	std::vector<int> pred_sem_;
	std::vector<int> pred_inst_;


	// Thread-safe error message access
	void set_error(std::string msg) {
		std::lock_guard<std::mutex> lk(error_mu_);
		error_msg_ = std::move(msg);
	}
	void clear_error() {
		std::lock_guard<std::mutex> lk(error_mu_);
		error_msg_.clear();
	}
	std::string get_error() const {
		std::lock_guard<std::mutex> lk(error_mu_);
		return error_msg_;
	}


	// Configuration management
	void load_parameter_file() {
		const std::vector<std::string> f = { "JSON Files (*.json)", "*.json" };
		std::string p = easy3d::dialog::open("Select Parameter File", "", f);
		if (!p.empty()) {
			load_config_from_path(p);
		}
	}

	void save_parameter_file() {
		if (!cfg_ok_) {
			g_log.add("[WARN] No configuration loaded.");
			return;
		}
		sync_cfg_from_bufs();
		const std::vector<std::string> f = { "JSON Files (*.json)", "*.json" };
		std::string p = easy3d::dialog::save("Save Parameter File", cfg_path_.string(), f);
		if (!p.empty()) {
			try {
				std::ofstream ofs(p);
				ofs << cfg_.dump(4);
				g_log.add("[INFO] Saved configuration to: " + p);
			} catch (const std::exception &e) {
				g_log.add(std::string("[ERROR] Save failed: ") + e.what());
			}
		}
	}

	void load_config_from_path(const std::string &path) {
		nlohmann::json j;
		try {
			std::ifstream ifs(path);
			if (!ifs.is_open()) {
				g_log.add("[WARN] Cannot open: " + path);
				return;
			}
			ifs >> j;
		} catch (const std::exception &e) {
			g_log.add(std::string("[ERROR] Load failed: ") + e.what());
			cfg_ok_ = false;
			return;
		}

		// Reject empty / non-object config before any destructive action
		if (j.is_null() || !j.is_object() || j.empty()) {
			g_log.add("[ERROR] Configuration file is empty: " + path);
			cfg_ok_ = false;
			return;
		}

		// Stage the config into memory. Wiping + ValidateConfig + SetLogFile are deferred
		// to the "Confirm Parameters" button, so loading a config is itself non-destructive.
		std::string base_output;
		if (j.contains("Output_Settings") && j["Output_Settings"].is_object()) {
			const auto &out = j["Output_Settings"];
			if (out.contains("Output_Folder_Path") && out["Output_Folder_Path"].is_string()) {
				base_output = out["Output_Folder_Path"].get<std::string>();
			}
		}

		try {
			cfg_ = std::move(j);
			base_output_path_ = base_output;
			std::string pc;
			if (cfg_.contains("Input_Settings") && cfg_["Input_Settings"].is_object() && cfg_["Input_Settings"].contains("Point_Cloud_File_Path") &&
				cfg_["Input_Settings"]["Point_Cloud_File_Path"].is_string()) {
				pc = cfg_["Input_Settings"]["Point_Cloud_File_Path"].get<std::string>();
			}
			if (!base_output.empty() && !pc.empty()) {
				std::filesystem::path out = std::filesystem::path(base_output) / std::filesystem::path(pc).stem();
				cfg_["Output_Settings"]["Output_Folder_Path"] = out.string();
			}
			cfg_path_ = path;
			cfg_ok_ = true;
			cfg_dirty_ = true;
			sync_bufs_from_cfg();
			g_log.add("[INFO] Loaded configuration from: " + path);
		} catch (const std::exception &e) {
			g_log.add(std::string("[ERROR] Apply config failed: ") + e.what());
			cfg_ok_ = false;
		}
	}

	void sync_bufs_from_cfg() {
		auto cp = [](char *d, size_t n, const nlohmann::json &j, const std::string &k) {
			if (j.contains(k) && j[k].is_string()) {
				strncpy(d, j[k].get<std::string>().c_str(), n - 1);
				d[n - 1] = 0;
			}
		};
		auto &in = cfg_["Input_Settings"];
		cp(buf_pc_path_, sizeof(buf_pc_path_), in, "Point_Cloud_File_Path");

		if (in.contains("Labels_Names")) {
			auto &ln = in["Labels_Names"];
			if (ln.contains("PLY_Format")) {
				auto &p = ln["PLY_Format"];
				cp(buf_sem_name_, sizeof(buf_sem_name_), p, "Semantic_Label_Name");
				cp(buf_inst_name_, sizeof(buf_inst_name_), p, "Instance_Label_Name");
			}
		}
		cp(buf_out_dir_, sizeof(buf_out_dir_), cfg_["Output_Settings"], "Output_Folder_Path");
	}

	void sync_cfg_from_bufs() {
		if (!cfg_ok_) {
			return;
		}
		auto &in = cfg_["Input_Settings"];
		in["Point_Cloud_File_Path"] = std::string(buf_pc_path_);

		auto &ply = in["Labels_Names"]["PLY_Format"];
		ply["Semantic_Label_Name"] = std::string(buf_sem_name_);
		ply["Instance_Label_Name"] = std::string(buf_inst_name_);

		cfg_["Output_Settings"]["Output_Folder_Path"] = std::string(buf_out_dir_);
	}


	// Triggered when the user finishes editing buf_out_dir_ (Enter / blur) or picks a folder via the dialog.
	// User input is treated as the new BASE; final = base / stem(pc) (or = base when pc is empty).
	// If the input already ends with stem(pc), strip it so re-typing the displayed final stays a no-op.
	// Wipe + validation are deferred to the "Confirm Parameters" button.
	void confirm_output_path_change_from_buf() {
		std::filesystem::path new_input = buf_out_dir_;

		std::string current_in_cfg;
		if (cfg_.contains("Output_Settings") && cfg_["Output_Settings"].is_object()) {
			const auto &out = cfg_["Output_Settings"];
			if (out.contains("Output_Folder_Path") && out["Output_Folder_Path"].is_string()) {
				current_in_cfg = out["Output_Folder_Path"].get<std::string>();
			}
		}

		// No actual edit
		if (new_input == std::filesystem::path(current_in_cfg)) {
			return;
		}

		// User blanked the field
		if (new_input.empty()) {
			if (cfg_.contains("Output_Settings") && cfg_["Output_Settings"].is_object()) {
				cfg_["Output_Settings"]["Output_Folder_Path"] = std::string();
			}
			base_output_path_.clear();
			cfg_dirty_ = true;
			return;
		}

		std::string pc_str;
		if (cfg_.contains("Input_Settings") && cfg_["Input_Settings"].is_object() && cfg_["Input_Settings"].contains("Point_Cloud_File_Path") &&
			cfg_["Input_Settings"]["Point_Cloud_File_Path"].is_string()) {
			pc_str = cfg_["Input_Settings"]["Point_Cloud_File_Path"].get<std::string>();
		}
		std::filesystem::path pc_stem = pc_str.empty() ? std::filesystem::path() : std::filesystem::path(pc_str).stem();
		std::filesystem::path new_base = (!pc_stem.empty() && new_input.filename() == pc_stem) ? new_input.parent_path() : new_input;
		std::filesystem::path new_final = pc_stem.empty() ? new_base : new_base / pc_stem;

		if (!cfg_.contains("Output_Settings") || !cfg_["Output_Settings"].is_object()) {
			cfg_["Output_Settings"] = nlohmann::json::object();
		}
		cfg_["Output_Settings"]["Output_Folder_Path"] = new_final.string();
		base_output_path_ = new_base.string();
		cfg_dirty_ = true;
		sync_bufs_from_cfg();
	}


	// Triggered when the user finishes editing buf_pc_path_ (Enter / blur) or picks a file via the dialog.
	// Stages the new pc into cfg and refreshes the final output (= base / stem(pc)) when a base
	// is known. Wipe + validation are deferred to the "Confirm Parameters" button.
	void confirm_pc_path_change_from_buf() {
		std::filesystem::path new_pc(buf_pc_path_);

		std::string current_pc;
		if (cfg_.contains("Input_Settings") && cfg_["Input_Settings"].is_object()) {
			const auto &in = cfg_["Input_Settings"];
			if (in.contains("Point_Cloud_File_Path") && in["Point_Cloud_File_Path"].is_string()) {
				current_pc = in["Point_Cloud_File_Path"].get<std::string>();
			}
		}

		// No actual change
		if (new_pc == std::filesystem::path(current_pc)) {
			return;
		}

		if (!cfg_.contains("Input_Settings") || !cfg_["Input_Settings"].is_object()) {
			cfg_["Input_Settings"] = nlohmann::json::object();
		}
		cfg_["Input_Settings"]["Point_Cloud_File_Path"] = new_pc.string();
		cfg_dirty_ = true;

		if (!new_pc.empty() && !base_output_path_.empty()) {
			const std::filesystem::path new_final = std::filesystem::path(base_output_path_) / new_pc.stem();
			cfg_["Output_Settings"]["Output_Folder_Path"] = new_final.string();
			sync_bufs_from_cfg();
		}
	}


	// Triggered by the "Confirm Parameters" button. Sole entry point for ValidateConfig:
	// flushes the GUI bufs into cfg_, asks the user before wiping the output folder, then
	// validates and binds the log file. All earlier path edits are pure staging.
	void confirm_parameters() {
		if (!cfg_ok_ || is_running_) {
			return;
		}
		sync_cfg_from_bufs();

		std::string out_path;
		if (cfg_.contains("Output_Settings") && cfg_["Output_Settings"].is_object() && cfg_["Output_Settings"].contains("Output_Folder_Path") &&
			cfg_["Output_Settings"]["Output_Folder_Path"].is_string()) {
			out_path = cfg_["Output_Settings"]["Output_Folder_Path"].get<std::string>();
		}
		if (out_path.empty()) {
			g_log.add("[ERROR] Output folder is not set.");
			return;
		}

		request_clear_output_confirmation(out_path, [this, out_path]() {
			try {
				configvalidator::ValidateConfig(cfg_);	// wipes output folder as a side effect
				Logger::Instance().SetLogFile(std::filesystem::path(out_path) / ".log");
				sync_bufs_from_cfg();
				cfg_dirty_ = false;
				g_log.add("[INFO] Parameters confirmed and validated.");
			} catch (const std::exception &e) {
				g_log.add(std::string("[ERROR] Validation failed: ") + e.what());
			}
		});
	}


	// Recreate the output folder. Returns true on success
	bool try_prepare_output_folder(const std::filesystem::path &path) {
		try {
			if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
				std::filesystem::remove_all(path);
			}
			std::filesystem::create_directories(path);
			return true;
		} catch (const std::exception &e) {
			std::string msg = "Failed to prepare output folder '" + path.string() + "': " + e.what();
			g_log.add("[ERROR] " + msg);
			set_error(std::move(msg));
			return false;
		}
	}


	// Confirm with the user that `path` is OK to wipe, then run `on_confirm`
	void request_clear_output_confirmation(const std::filesystem::path &path, std::function<void()> on_confirm,
										   std::function<void()> on_cancel = {}) {
		if (show_clear_output_popup_ || pending_after_confirm_) {
			g_log.add("[WARN] Confirmation already pending; ignoring duplicate request.");
			return;
		}

		pending_output_path_ = path;
		pending_output_preview_.clear();
		pending_output_item_count_ = 0;
		pending_output_has_contents_ = false;

		try {
			if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
				constexpr std::size_t preview_limit = 5;
				for (const auto &entry: std::filesystem::directory_iterator(path)) {
					if (pending_output_preview_.size() < preview_limit) {
						pending_output_preview_.push_back(entry.path().filename().string());
					}
					++pending_output_item_count_;
				}
				pending_output_has_contents_ = pending_output_item_count_ > 0;
			}
		} catch (const std::exception &e) {
			g_log.add(std::string("[WARN] Could not inspect output folder: ") + e.what());
		}

		const bool bypass = !pending_output_has_contents_ || dont_ask_again_session_;
		if (bypass) {
			if (on_confirm) {
				on_confirm();
			}
			return;
		}

		pending_after_confirm_ = std::move(on_confirm);
		pending_after_cancel_ = std::move(on_cancel);
		show_clear_output_popup_ = true;
	}


	void draw_clear_output_popup() {
		constexpr const char *popup_id = "Confirm Output Folder##clear_output";

		if (show_clear_output_popup_) {
			ImGui::OpenPopup(popup_id);
			show_clear_output_popup_ = false;
		}

		const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		ImGui::SetNextWindowSize(ImVec2(520, 0), ImGuiCond_Appearing);

		if (!ImGui::BeginPopupModal(popup_id, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
			return;
		}

		const bool danger = pending_output_has_contents_;

		// Header
		if (danger) {
			ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.20f, 1.0f), "[WARN] Output folder is NOT empty (%zu item%s).", pending_output_item_count_,
							   pending_output_item_count_ == 1 ? "" : "s");
		} else {
			ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1.0f), "[INFO] Output folder is empty and ready.");
		}
		ImGui::Separator();
		ImGui::Spacing();

		// Path
		ImGui::TextDisabled("Path:");
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(0.255f, 0.682f, 0.231f, 1), "%s", pending_output_path_.string().c_str());
		ImGui::Spacing();

		// Body
		if (danger) {
			ImGui::TextDisabled("Existing contents (preview):");
			ImGui::BeginChild("##preview", ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 5.5f), true);
			for (const auto &name: pending_output_preview_) {
				ImGui::BulletText("%s", name.c_str());
			}
			if (pending_output_item_count_ > pending_output_preview_.size()) {
				ImGui::TextDisabled("... and %zu more", pending_output_item_count_ - pending_output_preview_.size());
			}
			ImGui::EndChild();

			ImGui::Spacing();
			ImGui::TextWrapped("All existing contents will be PERMANENTLY DELETED before processing starts.");
		} else {
			ImGui::TextWrapped("Proceed to start processing?");
		}
		ImGui::Spacing();

		//  "Don't ask again" checkbox
		ImGui::Checkbox("Don't ask again this session", &dont_ask_again_session_);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip(
					"Skip this confirmation for the rest of this session.\n"
					"Existing output contents will still be deleted automatically.");
		}
		ImGui::Separator();

		// Buttons (centered)
		const float btn_w = 150.0f;
		const float spacing = ImGui::GetStyle().ItemSpacing.x;
		const float total_w = btn_w * 2.0f + spacing;
		ImGui::SetCursorPosX((ImGui::GetWindowSize().x - total_w) * 0.5f);

		// Red danger styling for the confirm button when content will be deleted.
		if (danger) {
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.20f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.30f, 0.30f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.55f, 0.10f, 0.10f, 1.0f));
		}
		// On the safe path, Enter confirms; on the danger path require an explicit click.
		if (!danger && ImGui::IsWindowAppearing()) {
			ImGui::SetKeyboardFocusHere();
		}

		const char *confirm_label = danger ? "Delete & Continue" : "Continue";
		const bool confirm_pressed = ImGui::Button(confirm_label, ImVec2(btn_w, 0)) || (!danger && ImGui::IsKeyPressed(ImGuiKey_Enter));
		if (confirm_pressed) {
			auto confirm_cb = std::move(pending_after_confirm_);
			pending_after_confirm_ = nullptr;
			pending_after_cancel_ = nullptr;
			if (confirm_cb) {
				confirm_cb();
			}
			ImGui::CloseCurrentPopup();
		}
		if (danger) {
			ImGui::PopStyleColor(3);
		}

		ImGui::SameLine();
		if (ImGui::Button("Cancel", ImVec2(btn_w, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
			auto cancel_cb = std::move(pending_after_cancel_);
			pending_after_confirm_ = nullptr;
			pending_after_cancel_ = nullptr;
			if (cancel_cb) {
				cancel_cb();
			}
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}


	// About dialog
	void draw_about() {
		ImGui::SetNextWindowPos(ImVec2(width() * 0.5f, height() * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		ImGui::Begin("About", &show_about_, ImGuiWindowFlags_NoResize);
		ImGui::Text("SkelSeg");
		ImGui::Separator();
		ImGui::Text("Plant Skeleton Extraction & Segmentation");
		ImGui::End();
	}


	void draw_help() {
		ImGui::SetNextWindowPos(ImVec2(width() * 0.5f, height() * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);
		ImGui::Begin("Viewer usage", &show_help_, ImGuiWindowFlags_NoResize);
		ImGui::Text("Manual");
		ImGui::Separator();
		ImGui::TextUnformatted(easy3d_usage_.c_str());
		ImGui::End();
	}


	// Parameter editing panel
	void draw_parameter_panel() {
		ImGui::SetNextWindowPos(ImVec2(3, menu_height_ + 35), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(380, 560), ImGuiCond_FirstUseEver);

		if (!ImGui::Begin("Parameters", &show_param_panel_)) {
			ImGui::End();
			return;
		}

		if (!cfg_ok_) {
			ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No configuration loaded.");
			if (ImGui::Button("Load Configuration File")) {
				load_parameter_file();
			}
			ImGui::SameLine();
			if (ImGui::Button("Load Default Configuration File")) {
				load_config_from_path(ROOT_DIR "/resources/default_configure.json");
			}

			ImGui::End();
			return;
		}

		// Toolbar
		if (ImGui::Button("Load")) {
			load_parameter_file();
		}
		ImGui::SameLine();
		if (ImGui::Button("Save")) {
			save_parameter_file();
		}
		ImGui::SameLine();
		if (ImGui::Button("Reload") && !cfg_path_.empty()) {
			load_config_from_path(cfg_path_.string());
		}
		ImGui::SameLine();
		if (ImGui::Button("Load Default Parameters")) {
			load_config_from_path(ROOT_DIR "/resources/default_configure.json");
		}
		ImGui::Separator();

		ImGui::PushItemWidth(ImGui::GetFontSize() * 12);
		// Input Settings
		if (ImGui::CollapsingHeader("Input Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
			auto &in = cfg_["Input_Settings"];

			ImGui::InputText("Input Point Cloud", buf_pc_path_, sizeof(buf_pc_path_));
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", buf_pc_path_);
			}
			bool needs_pc_confirm = ImGui::IsItemDeactivatedAfterEdit();
			ImGui::SameLine();
			if (ImGui::SmallButton("...##pc")) {
				const std::vector<std::string> f = { "Input Point Cloud (*.ply *.xyz *.txt)", "*.ply *.xyz *.txt" };
				std::string p = easy3d::dialog::open("Select Input Point Cloud", "", f);
				if (!p.empty()) {
					strncpy(buf_pc_path_, p.c_str(), sizeof(buf_pc_path_) - 1);
					buf_pc_path_[sizeof(buf_pc_path_) - 1] = 0;
					needs_pc_confirm = true;
				}
			}
			if (needs_pc_confirm) {
				confirm_pc_path_change_from_buf();
			}

			std::filesystem::path input_file_path(buf_pc_path_);
			bool wl = in.value("With_Labels", false);
			bool is_ply = (input_file_path.extension() == ".ply");

			if (!is_ply) {	// Default format: x, y ,z
				auto &xyz_txt = in["Labels_Names"]["TXT_XYZ_Format"];
				int total_cols = xyz_txt.value("Total_Columns", 3);
				xyz_txt["Total_Columns"] = total_cols;
			}

			if (ImGui::Checkbox("With Labels", &wl)) {
				in["With_Labels"] = wl;
			}

			if (wl) {
				if (is_ply) {
					if (ImGui::TreeNodeEx("PLY Format Labels", ImGuiTreeNodeFlags_DefaultOpen)) {
						ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
						ImGui::InputText("Semantic Label Name", buf_sem_name_, sizeof(buf_sem_name_));
						ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
						ImGui::InputText("Instance Label Name", buf_inst_name_, sizeof(buf_inst_name_));
						ImGui::TreePop();
					}
				} else {
					// TODO: support xyz/txt input with labels
					// auto &xyz_txt = in["Labels_Names"]["TXT_XYZ_Format"];
					// int total_cols = xyz_txt.value("Total_Columns", 4);
					// if (ImGui::InputInt("Total Columns (Start from 0)", &total_cols)) {
					// 	xyz_txt["Total_Columns"] = total_cols;
					// }
					// if (ImGui::TreeNodeEx("XYZ/TXT Format Labels", ImGuiTreeNodeFlags_DefaultOpen)) {
					// 	int sem_cols = xyz_txt.value("Semantic_Labels_Index", 3);
					// 	int ins_cols = xyz_txt.value("Instance_Labels_Index", 4);
					// 	if (ImGui::InputInt("Semantic Columns Index (Start from 0)", &sem_cols)) {
					// 		xyz_txt["Semantic_Labels_Index"] = sem_cols;
					// 	}
					// 	if (ImGui::InputInt("Instance Columns Index (Start from 0)", &ins_cols)) {
					// 		xyz_txt["Instance_Labels_Index"] = ins_cols;
					// 	}
					// 	ImGui::TreePop();
					// }
				}
			}
		}

		// Preprocess
		if (ImGui::CollapsingHeader("Preprocess")) {
			auto &pr = cfg_["Preprocess"];
			int dn = pr.value("Down_Sample_Number", 10240);
			if (ImGui::InputInt("Down Sample Number", &dn)) {
				if (dn > 0) {
					pr["Down_Sample_Number"] = dn;
				}
			}

			float dl = static_cast<float>(pr.value("Normalize_AABB_Length", 1.60));
			if (ImGui::InputFloat("Normalize AABB", &dl, 0.01f, 0.1f, "%.3f"))
				if (dl > 0) {
					pr["Normalize_AABB_Length"] = static_cast<double>(dl);
				}
		}

		// Constrained Laplacian Operator
		if (ImGui::CollapsingHeader("Constrained Laplacian Operator", ImGuiTreeNodeFlags_DefaultOpen)) {
			auto &cl = cfg_["Constrained_Laplacian_Operator"];

			bool knn = cl.value("Use_KNN_Search", true);
			bool rad = cl.value("Use_Radius_Search", false);
			if (knn == false && rad == false) {
				knn = true;
			}
			if (ImGui::RadioButton("KNN Search", knn)) {
				cl["Use_KNN_Search"] = true;
				cl["Use_Radius_Search"] = false;
				knn = true;
				rad = false;
			}
			ImGui::SameLine();
			if (ImGui::RadioButton("Radius Search", rad)) {
				cl["Use_KNN_Search"] = false;
				cl["Use_Radius_Search"] = true;
				knn = false;
				rad = true;
			}
			ImGui::Separator();

			if (knn == true && rad == false) {
				// KNN Parameters
				ImGui::TextDisabled("KNN Parameters");
				int ik = cl.value("Initial_k", 8);
				if (ImGui::InputInt("Initial k", &ik)) {
					if (ik > 0) {
						cl["Initial_k"] = ik;
					}
				}
				int dk = cl.value("Delta_k", 4);
				if (ImGui::InputInt("Delta k", &dk)) {
					if (dk > 0) {
						cl["Delta_k"] = dk;
					}
				}
				int mk = cl.value("Max_k", 32);
				if (ImGui::InputInt("Max k", &mk)) {
					if (mk > 0) {
						cl["Max_k"] = mk;
					}
				}
			} else {
				// Radius Parameters
				ImGui::TextDisabled("Radius Parameters");
				float ir = static_cast<float>(cl.value("Initial_Radius_Search_Ratio", 0.015));
				if (ImGui::InputFloat("Initial Radius Ratio", &ir, 0.001f, 0.01f, "%.4f")) {
					if (ir > 0) {
						cl["Initial_Radius_Search_Ratio"] = static_cast<double>(ir);
					}
				}
				float dr = static_cast<float>(cl.value("Delta_Radius_Search_Ratio", 0.005));
				if (ImGui::InputFloat("Delta Radius Ratio", &dr, 0.001f, 0.01f, "%.4f")) {
					if (dr > 0) {
						cl["Delta_Radius_Search_Ratio"] = static_cast<double>(dr);
					}
				}
				float mr = static_cast<float>(cl.value("Min_Radius_Search_Ratio", 0.005));
				if (ImGui::InputFloat("Min Radius Ratio", &mr, 0.001f, 0.01f, "%.4f")) {
					if (mr > 0) {
						cl["Min_Radius_Search_Ratio"] = static_cast<double>(mr);
					}
				}
			}
		}

		// Adaptive Contraction
		if (ImGui::CollapsingHeader("Adaptive Contraction")) {
			auto &ac = cfg_["Adaptive_Contraction"];
			float st = static_cast<float>(ac.value("Smooth_Sigma_Threshold", 0.90));
			if (ImGui::DragFloat("Sigma Threshold", &st, 0.01f, 0.0f, 1.0f, "%.2f")) {
				ac["Smooth_Sigma_Threshold"] = static_cast<double>(st);
			}
			float sr = static_cast<float>(ac.value("Sigma_Sphere_Radius_Ratio", 0.015));
			if (ImGui::InputFloat("Sigma Radius Ratio", &sr, 0.001f, 0.01f, "%.4f")) {
				if (sr > 0) {
					ac["Sigma_Sphere_Radius_Ratio"] = static_cast<double>(sr);
				}
			}
			float md = static_cast<float>(ac.value("Max_Distance_Ratio", 0.005));
			if (ImGui::InputFloat("Max Distance Ratio", &md, 0.001f, 0.01f, "%.4f")) {
				if (md > 0) {
					ac["Max_Distance_Ratio"] = static_cast<double>(md);
				}
			}
		}

		// Terminate Condition
		if (ImGui::CollapsingHeader("Terminate Condition")) {
			auto &tc = cfg_["Terminate_Condition"];
			int mi = tc.value("Max_Iteration", 25);
			if (ImGui::InputInt("Max Iteration", &mi)) {
				if (mi > 0) {
					tc["Max_Iteration"] = mi;
				}
			}
			float ct = static_cast<float>(tc.value("Convergence_Threshold", 1e-4));
			if (ImGui::InputFloat("Convergence Threshold", &ct, 0.0f, 0.0f, "%.2e")) {
				if (ct > 0) {
					tc["Convergence_Threshold"] = static_cast<double>(ct);
				}
			}
		}

		// Skeleton Building
		if (ImGui::CollapsingHeader("Skeleton Building")) {
			auto &sb = cfg_["Skeleton_Building"];
			float ds = static_cast<float>(sb.value("Down_Sample_Ratio", 0.10));
			if (ImGui::SliderFloat("Down Sample Ratio", &ds, 0.01f, 1.0f, "%.3f")) {
				sb["Down_Sample_Ratio"] = static_cast<double>(ds);
			}
			float lr = static_cast<float>(sb.value("LOP_Sphere_Radius_Ratio", 0.045));
			if (ImGui::InputFloat("LOP Radius Ratio", &lr, 0.001f, 0.01f, "%.4f")) {
				if (lr > 0) {
					sb["LOP_Sphere_Radius_Ratio"] = static_cast<double>(lr);
				}
			}
			float nb = static_cast<float>(sb.value("Noise_Branch_Length_Ratio", 0.001));
			if (ImGui::InputFloat("Noise Branch Ratio", &nb, 0.0001f, 0.001f, "%.5f")) {
				if (nb > 0) {
					sb["Noise_Branch_Length_Ratio"] = static_cast<double>(nb);
				}
			}
		}

		// Output Settings
		if (ImGui::CollapsingHeader("Output Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::InputText("Output Folder", buf_out_dir_, sizeof(buf_out_dir_));
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", buf_out_dir_);
			}
			bool needs_confirm = ImGui::IsItemDeactivatedAfterEdit();
			ImGui::SameLine();
			if (ImGui::SmallButton("...##od")) {
				std::string d = easy3d::dialog::open_folder("Select Output Folder");
				if (!d.empty()) {
					strncpy(buf_out_dir_, d.c_str(), sizeof(buf_out_dir_) - 1);
					buf_out_dir_[sizeof(buf_out_dir_) - 1] = 0;
					needs_confirm = true;
				}
			}
			if (needs_confirm) {
				confirm_output_path_change_from_buf();
			}

			std::string fmt = cfg_["Output_Settings"].value("Output_PLY_File_DataFormat", "Binary");
			int fi = (fmt == "ASCII") ? 1 : 0;
			const char *items[] = { "Binary", "ASCII" };
			if (ImGui::Combo("PLY Format", &fi, items, 2)) {
				cfg_["Output_Settings"]["Output_PLY_File_DataFormat"] = std::string(items[fi]);
			}
		}
		ImGui::PopItemWidth();

		// Confirm Parameters: the only entry point that runs ValidateConfig and prepares the output folder
		ImGui::Separator();
		ImGui::Spacing();
		const float btn_w = 220.0f;
		ImGui::SetCursorPosX((ImGui::GetWindowSize().x - btn_w) * 0.5f);
		const bool can_confirm = !is_running_;
		if (!can_confirm) {
			ImGui::BeginDisabled();
		}
		if (ImGui::Button("Confirm Parameters", ImVec2(btn_w, 32))) {
			confirm_parameters();
		}
		if (!can_confirm) {
			ImGui::EndDisabled();
		}
		ImGui::Spacing();

		ImGui::End();
	}


	// Pipeline control panel
	void draw_pipeline_panel() {
		ImGui::SetNextWindowPos(ImVec2(static_cast<float>(width()) - 323, menu_height_ + 35), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(320, 400), ImGuiCond_FirstUseEver);

		if (!ImGui::Begin("Pipeline Control", &show_pipeline_panel_)) {
			ImGui::End();
			return;
		}

		if (!cfg_ok_) {
			ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Load a configuration file first.");
			ImGui::End();
			return;
		}

		// Status
		if (is_running_) {
			int rs = running_stage_.load();
			ImGui::TextColored(ImVec4(1, 0.8f, 0, 1), "Running: %s", (rs >= 0 && rs <= kStageCount) ? kStageNames[rs] : "...");
			ImGui::SameLine();
			const char *sp = "|/-\\";
			ImGui::Text("%c", sp[static_cast<int>(ImGui::GetTime() * 8) % 4]);
		} else {
			int cs = completed_.load();
			ImGui::TextColored(ImVec4(0.255f, 0.682f, 0.231f, 1), "Completed: %s", (cs >= 0 && cs <= kStageCount) ? kStageNames[cs] : "None");
		}

		const std::string err = get_error();
		if (!err.empty()) {
			ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Error: %s", err.c_str());
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", err.c_str());
			}
		}

		ImGui::Separator();

		// Per-step buttons
		auto step_btn = [this](const char *label, Stage stg, Stage prereq) {
			bool ok = !is_running_ && !cfg_dirty_ && completed_.load() >= static_cast<int>(prereq);
			if (!ok) {
				ImGui::BeginDisabled();
			}
			bool done = completed_.load() >= static_cast<int>(stg);
			if (done) {
				ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.2f, 0.65f));
			}
			if (ImGui::Button(label, ImVec2(-1, 0))) {
				launch_step(stg);
			}
			if (done) {
				ImGui::PopStyleColor();
			}
			if (!ok) {
				ImGui::EndDisabled();
			}
			if (cfg_dirty_ && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
				ImGui::SetTooltip("Click 'Confirm Parameters' first.");
			}
		};

		step_btn("1. Load Point Cloud", Stage::LOAD_POINT_CLOUD, Stage::NONE);
		step_btn("2. Preprocess", Stage::PREPROCESS, Stage::LOAD_POINT_CLOUD);
		step_btn("3. Laplacian Contraction", Stage::LAPLACIAN, Stage::PREPROCESS);
		step_btn("4. Skeleton Downsample", Stage::SKEL_DOWNSAMPLE, Stage::LAPLACIAN);
		step_btn("5. LOP Calibration", Stage::LOP_CALIBRATION, Stage::SKEL_DOWNSAMPLE);
		step_btn("6. Build Graph", Stage::BUILD_GRAPH, Stage::LOP_CALIBRATION);
		step_btn("7. Compute MST", Stage::COMPUTE_MST, Stage::BUILD_GRAPH);
		step_btn("8. Prune MST", Stage::PRUNE_MST, Stage::COMPUTE_MST);
		step_btn("9. Segment Skeleton", Stage::SEGMENT_SKELETON, Stage::PRUNE_MST);
		step_btn("10. Assign Labels", Stage::ASSIGN_LABELS, Stage::SEGMENT_SKELETON);

		ImGui::Separator();

		bool ok = !is_running_ && cfg_ok_ && !cfg_dirty_;
		if (!ok) {
			ImGui::BeginDisabled();
		}
		if (ImGui::Button("Run All Steps", ImVec2(-1, 32))) {
			run_all();
		}
		if (!ok) {
			ImGui::EndDisabled();
		}
		if (cfg_dirty_ && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
			ImGui::SetTooltip("Click 'Confirm Parameters' first.");
		}

		ImGui::Separator();

		if (!is_running_) {
			if (ImGui::Button("Reset Pipeline", ImVec2(-1, 0))) {
				reset_pipeline();
			}
		}

		ImGui::End();
	}


	// Log panel
	void draw_log_panel() {
		ImGui::SetNextWindowPos(ImVec2(197, static_cast<float>(height()) - 203), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(static_cast<float>(width()) - 200, 200), ImGuiCond_FirstUseEver);

		if (!ImGui::Begin("Log", &show_log_panel_)) {
			ImGui::End();
			return;
		}

		if (ImGui::SmallButton("Clear")) {
			g_log.clear();
		}
		ImGui::SameLine();
		ImGui::TextDisabled("Log Output");
		ImGui::Separator();

		ImGui::BeginChild("##logscroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

		const bool need_scroll = g_log.render([](const std::string &l) {
			if (l.find("[ERROR]") != std::string::npos) {
				ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", l.c_str());
			} else if (l.find("[WARN") != std::string::npos) {
				ImGui::TextColored(ImVec4(1, 0.8f, 0, 1), "%s", l.c_str());
			} else if (l.find("[INFO]") != std::string::npos) {
				ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1), "%s", l.c_str());
			} else {
				ImGui::TextUnformatted(l.c_str());
			}
		});

		if (need_scroll) {
			ImGui::SetScrollHereY(1.0f);
		}

		ImGui::EndChild();
		ImGui::End();
	}


	// Layer Manager panel
	void draw_layer_manager() {
		ImGui::SetNextWindowPos(ImVec2(static_cast<float>(width()) - 626, menu_height_ + 3), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);

		if (!ImGui::Begin("Layer Manager", &show_layer_manager_)) {
			ImGui::End();
			return;
		}

		const auto &all_models = models();

		if (all_models.empty() && drawables().empty()) {
			ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "No models loaded.");
			ImGui::End();
			return;
		}

		// Toolbar
		if (ImGui::SmallButton("Show All")) {
			for (const auto &m: all_models) {
				if (m->renderer()) {
					m->renderer()->set_visible(true);
				}
			}
			for (const auto &d: drawables()) {
				d->set_visible(true);
			}
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("Hide All")) {
			for (const auto &m: all_models) {
				if (m->renderer()) {
					m->renderer()->set_visible(false);
				}
			}
			for (const auto &d: drawables()) {
				d->set_visible(false);
			}
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("Invert")) {
			for (const auto &m: all_models) {
				if (m->renderer()) {
					m->renderer()->set_visible(!m->renderer()->is_visible());
				}
			}
			for (const auto &d: drawables()) {
				d->set_visible(!d->is_visible());
			}
		}
		ImGui::Separator();

		// Model list
		easy3d::Model *cur = current_model();
		int model_to_delete = -1;

		ImGui::BeginChild("##model_list", ImVec2(0, 0), false);

		for (int mi = 0; mi < static_cast<int>(all_models.size()); ++mi) {
			auto *model = all_models[mi].get();
			auto *ren = model->renderer();
			if (!ren) {
				continue;
			}

			ImGui::PushID(mi);

			// Visibility checkbox
			bool vis = ren->is_visible();
			if (ImGui::Checkbox("##vis", &vis)) {
				ren->set_visible(vis);
			}
			ImGui::SameLine();

			// Model type icon
			const char *icon = "[?]";
			if (dynamic_cast<easy3d::PointCloud *>(model)) {
				icon = "[P]";
			} else if (dynamic_cast<easy3d::Graph *>(model)) {
				icon = "[G]";
			} else if (dynamic_cast<easy3d::SurfaceMesh *>(model)) {
				icon = "[M]";
			}
			ImGui::TextColored(ImVec4(0.5f, 0.7f, 1.0f, 1.0f), "%s", icon);
			ImGui::SameLine();

			// Selectable model name (highlight current model)
			bool is_current = (model == cur);
			ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_AllowOverlap;
			if (is_current) {
				flags |= ImGuiTreeNodeFlags_Selected;
			}

			// Check if model has any drawables to show as tree children
			bool has_drawables = !ren->points_drawables().empty() || !ren->lines_drawables().empty() || !ren->triangles_drawables().empty();
			if (!has_drawables) {
				flags |= ImGuiTreeNodeFlags_Leaf;
			}

			bool node_open = ImGui::TreeNodeEx(model->name().c_str(), flags);

			// Click to select as current model
			if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
				// Set current model by iterating models
				for (const auto &all_model: all_models) {
					if (all_model.get() == model) {
						auto *ren_k = all_model->renderer();
						if (ren_k) {
							ren_k->set_selected(true);
						}
					} else {
						auto *ren_k = all_model->renderer();
						if (ren_k) {
							ren_k->set_selected(false);
						}
					}
				}
			}

			// Right-click context menu
			if (ImGui::BeginPopupContextItem("##model_ctx")) {
				if (ImGui::MenuItem("Show Only This")) {
					for (const auto &m: all_models) {
						if (m->renderer()) {
							m->renderer()->set_visible(m.get() == model);
						}
					}
				}
				if (ImGui::MenuItem(vis ? "Hide" : "Show")) {
					ren->set_visible(!vis);
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Fit to View")) {
					fit_screen(model);
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Delete")) {
					model_to_delete = mi;
				}
				ImGui::EndPopup();
			}

			// Drawable children
			if (node_open) {
				auto draw_drawable_row = [](const char *type_tag, auto &drawable_list) {
					for (const auto &d: drawable_list) {
						ImGui::PushID(d.get());
						bool dvis = d->is_visible();
						if (ImGui::Checkbox("##dvis", &dvis)) {
							d->set_visible(dvis);
						}
						ImGui::SameLine();
						ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", type_tag);
						ImGui::SameLine();
						ImGui::TextUnformatted(d->name().c_str());
						ImGui::PopID();
					}
				};

				draw_drawable_row("Pts ", ren->points_drawables());
				draw_drawable_row("Lns ", ren->lines_drawables());
				draw_drawable_row("Tri ", ren->triangles_drawables());
				ImGui::TreePop();
			}

			ImGui::PopID();
		}

		// Standalone drawables
		const auto &standalone = drawables();
		if (!standalone.empty()) {
			ImGui::Separator();
			ImGui::TextColored(ImVec4(0.8f, 0.6f, 0.2f, 1.0f), "Standalone Drawables");
			for (int di = 0; di < static_cast<int>(standalone.size()); ++di) {
				auto *d = standalone[di].get();
				ImGui::PushID(1000 + di);
				bool dvis = d->is_visible();
				if (ImGui::Checkbox("##sdvis", &dvis)) {
					d->set_visible(dvis);
				}
				ImGui::SameLine();
				ImGui::TextUnformatted(d->name().c_str());
				ImGui::PopID();
			}
		}

		ImGui::EndChild();
		ImGui::End();

		// Deferred model deletion (outside the iteration loop)
		if (model_to_delete >= 0 && model_to_delete < static_cast<int>(all_models.size())) {
			delete_model(all_models[model_to_delete].get());
		}
	}


	// Pipeline execution  (background thread)
	void launch_step(Stage stg) {
		if (is_running_) {
			return;
		}
		sync_cfg_from_bufs();
		clear_error();
		is_running_ = true;
		running_stage_ = static_cast<int>(stg);

		if (worker_.joinable()) {
			worker_.join();
		}

		worker_ = std::thread([this, stg]() {
			try {
				exec(stg);
				completed_ = static_cast<int>(stg);
				{
					std::lock_guard<std::mutex> lk(vis_mu_);
					vis_pending_.push_back(stg);
				}
			} catch (const std::exception &e) {
				set_error(e.what());
				g_log.add(std::string("[ERROR] ") + e.what());
			}
			is_running_ = false;
		});
	}


	void run_all() {
		if (is_running_) {
			return;
		}
		sync_cfg_from_bufs();
		clear_error();
		is_running_ = true;
		running_stage_ = static_cast<int>(Stage::LOAD_POINT_CLOUD);

		if (worker_.joinable()) {
			worker_.join();
		}

		worker_ = std::thread([this]() {
			try {
				Logger::Instance().PrintTitle();
				Logger::Instance().Log("SkelSeg GUI - Run All");

				for (int s = static_cast<int>(Stage::LOAD_POINT_CLOUD); s <= static_cast<int>(Stage::ASSIGN_LABELS); ++s) {
					running_stage_ = s;
					exec(static_cast<Stage>(s));
					completed_ = s;
					{
						std::lock_guard<std::mutex> lk(vis_mu_);
						vis_pending_.push_back(static_cast<Stage>(s));
					}
				}

				g_log.add("[INFO] Full pipeline completed successfully!");
				Logger::Instance().Log("SkelSeg GUI - Pipeline Complete!");
			} catch (const std::exception &e) {
				set_error(e.what());
				g_log.add(std::string("[ERROR] Pipeline failed: ") + e.what());
			}
			is_running_ = false;
		});
	}


	void exec(Stage stg) {
		std::filesystem::path out = cfg_["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();

		switch (stg) {
			case Stage::LOAD_POINT_CLOUD: {
				Logger::Instance().PrintTitle();
				Logger::Instance().Log("SkelSeg GUI - Run Step by Step");

				g_log.add("[INFO] Loading point cloud...");
				original_cloud_ = tool::utility::Vector2Matrix(tool::io::LoadPointCloud(cfg_));
				g_log.add("[INFO] Loaded " + std::to_string(original_cloud_.rows()) + " points");
				break;
			}
			case Stage::PREPROCESS: {
				g_log.add("[INFO] Normalizing and down point cloud...");
				tool::preprocess::PreparePointCloud(input_cloud_, cfg_);
				g_log.add("[INFO] Downsampled to " + std::to_string(input_cloud_.rows()) + " points");
				break;
			}
			case Stage::LAPLACIAN: {
				g_log.add("[INFO] Running Laplacian contraction...");
				Skeleton skel(input_cloud_, cfg_);
				lap_skeleton_ = skel.GetLaplacianSkeleton();
				g_log.add("[INFO] Contraction done: " + std::to_string(lap_skeleton_.rows()) + " skeleton pts");
				break;
			}
			case Stage::SKEL_DOWNSAMPLE: {
				g_log.add("[INFO] Skeleton downsampling...");
				auto n = static_cast<size_t>(std::round(cfg_["Skeleton_Building"]["Down_Sample_Ratio"].get<double>() * lap_skeleton_.rows()));
				std::vector<size_t> idx;
				std::tie(fps_points_, idx) = tool::utility::FarthestPointDownSample(lap_skeleton_, n);
				tool::io::SavePointCloudToPLY(fps_points_, out / "2_FPS-Downsampled.ply");
				g_log.add("[INFO] Downsampled to " + std::to_string(fps_points_.rows()) + " points");
				break;
			}
			case Stage::LOP_CALIBRATION: {
				g_log.add("[INFO] LOP calibration...");
				lop_points_ = LOPCalibrate(input_cloud_, fps_points_, cfg_);
				tool::io::SavePointCloudToPLY(lop_points_, out / "3_LOP-Calibrated.ply");
				g_log.add("[INFO] Calibrated " + std::to_string(lop_points_.rows()) + " skeleton points");
				break;
			}
			case Stage::BUILD_GRAPH: {
				g_log.add("[INFO] Building skeleton graph...");
				graph_obj_ = std::make_unique<::Graph>(lop_points_, cfg_);
				initial_graph_ = graph_obj_->GetInitialGraph();
				tool::io::SaveSkeletonGraphToPLY(initial_graph_, out / "4_Initial-Graph.ply");
				g_log.add("[INFO] Graph built");
				break;
			}
			case Stage::COMPUTE_MST: {
				g_log.add("[INFO] Computing MST...");
				mst_graph_ = graph_obj_->GetMST();
				tool::io::SaveSkeletonGraphToPLY(mst_graph_, out / "5_MST-Raw.ply");
				g_log.add("[INFO] MST computed");
				break;
			}
			case Stage::PRUNE_MST: {
				g_log.add("[INFO] Pruning MST...");
				pruned_mst_ = graph_obj_->GetPrunedMST();
				tool::io::SaveSkeletonGraphToPLY(pruned_mst_, out / "6_MST-Pruned.ply");
				g_log.add("[INFO] MST pruned");
				break;
			}
			case Stage::SEGMENT_SKELETON: {
				g_log.add("[INFO] Segmenting skeleton...");
				std::tie(skel_sem_, skel_inst_) = graph_obj_->SegmentSkeleton();
				tool::io::SaveSkeletonGraphToPLY(pruned_mst_, out / "7_MST-Segmented.ply", skel_sem_, skel_inst_);
				g_log.add("[INFO] Skeleton segmented");
				break;
			}
			case Stage::ASSIGN_LABELS: {
				g_log.add("[INFO] Projecting labels...");
				pred_inst_ = tool::utility::NearestProjectFromBoostVertices(pruned_mst_, input_cloud_, skel_inst_, "vertex");
				pred_sem_.resize(input_cloud_.rows());
				for (int i = 0; i < static_cast<int>(pred_inst_.size()); ++i) {
					pred_sem_[i] = (pred_inst_[i] == -1) ? 0 : 1;
				}

				auto stem = cfg_["Input_Settings"]["Point_Cloud_File_Path"].get<std::filesystem::path>().stem().string();
				std::pair<std::string, std::string> ln = { "pred-semantic", "pred-instance" };
				tool::io::SavePointCloudToPLY(input_cloud_, out / (stem + "_Result.ply"), pred_sem_, pred_inst_, ln);
				tool::io::SaveJSONFile(cfg_, out / "configure.json");
				g_log.add("[INFO] Labels projected, result saved.");
				break;
			}
			default:
				break;
		}
	}


	// Visualization (main thread only)
	void drain_pending_visualizations() {
		std::vector<Stage> todo;
		{
			std::lock_guard<std::mutex> lk(vis_mu_);
			todo.swap(vis_pending_);
		}
		for (auto s: todo) {
			visualize(s);
		}

		if (!is_running_ && worker_.joinable()) {
			worker_.join();
		}
	}


	void visualize(Stage stg) {
		std::filesystem::path out = cfg_["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
		switch (stg) {
			case Stage::PREPROCESS:
				show_layer_manager_ = true;
				add_pc("Original Point Cloud", out / "0_Original.ply", { 0.6f, 0.6f, 0.6f, 1.0f }, false);
				if (buf_inst_name_[0] != '\0') {
					add_labeled_pc("Input Point Cloud", out / "1_Input.ply", std::string("v:") + buf_inst_name_);
				} else if (buf_sem_name_[0] != '\0') {
					add_labeled_pc("Input Point Cloud", out / "1_Input.ply", std::string("v:") + buf_sem_name_);
				} else {
					add_pc("Input Point Cloud", out / "1_Input.ply", { 0.6f, 0.6f, 0.6f, 1.0f });
				}
				break;
			case Stage::LAPLACIAN:
				add_pc("Laplacian Skeleton", find_latest_cpts(out / ".iterations"), { 1.0f, 0.2f, 0.2f, 1.0f });
				break;
			case Stage::SKEL_DOWNSAMPLE:
				add_pc("FPS Downsampled Skeleton", out / "2_FPS-Downsampled.ply", { 0.2f, 1.0f, 0.2f, 1.0f });
				break;
			case Stage::LOP_CALIBRATION:
				add_pc("LOP Calibrated", out / "3_LOP-Calibrated.ply", { 0.2f, 0.2f, 1.0f, 1.0f });
				break;
			case Stage::BUILD_GRAPH:
				add_graph("Initial Graph", out / "4_Initial-Graph.ply");
				break;
			case Stage::COMPUTE_MST:
				add_graph("Raw MST", out / "5_MST-Raw.ply");
				break;
			case Stage::PRUNE_MST:
				add_graph("Pruned MST", out / "6_MST-Pruned.ply");
				break;
			case Stage::SEGMENT_SKELETON:
				add_labeled_graph("Segmented Skeleton", out / "7_MST-Segmented.ply");
				break;
			case Stage::ASSIGN_LABELS:
				add_labeled_pc("Result", find_result_ply(out), "v:pred-instance");
				break;
			default:
				break;
		}
	}


	// Place the model in the empty space within default opened windows
	void center_view() {
		std::vector<float> camera_state = { 3.62, 0.37, -0.2, -0.5, -0.5, -0.5, -0.5 };
		easy3d::vec3 position = easy3d::vec3(camera_state[0], camera_state[1], camera_state[2]);
		easy3d::quat orientation = { camera_state[3], camera_state[4], camera_state[5], camera_state[6] };
		camera()->setPosition(position);
		camera()->setOrientation(orientation);
	}


	void add_pc(const std::string &name, const std::string &ply_file, easy3d::vec4 color, bool visual = true) {
		easy3d::Model *pc = add_model(ply_file);
		if (!pc) {
			g_log.add("[ERROR] Failed to load: " + ply_file);
			return;
		}
		pc->set_name(name);
		if (auto *points = pc->renderer()->get_points_drawable("vertices", false)) {
			points->set_uniform_coloring(color);
		}
		pc->renderer()->set_visible(visual);
		center_view();
	}


	void add_graph(const std::string &name, const std::string &ply_file) {
		easy3d::Model *graph = add_model(ply_file);
		if (!graph) {
			g_log.add("[ERROR] Failed to load: " + ply_file);
			return;
		}
		graph->set_name(name);
		center_view();
	}


	void add_labeled_pc(const std::string &name, const std::string &ply_file, const std::string &label_name) {
		easy3d::Model *pc = add_model(ply_file);
		if (!pc) {
			g_log.add("[ERROR] Failed to load: " + ply_file);
			return;
		}
		pc->set_name(name);
		if (auto *points = pc->renderer()->get_points_drawable("vertices", false)) {
			const easy3d::Texture *tex = easy3d::TextureManager::request(ROOT_DIR "/resources/colormap.png", 42);
			try {
				points->set_scalar_coloring(easy3d::State::VERTEX, label_name, tex, 0.0f, 0.0f);
			} catch (const std::exception &e) {
				g_log.add(std::string("[WARN] Scalar coloring failed (") + label_name + "): " + e.what());
				points->set_uniform_coloring({ 0.6f, 0.6f, 0.6f, 1.0f });
			}
		}
		center_view();
	}


	void add_labeled_graph(const std::string &name, const std::string &ply_file) {
		easy3d::Model *graph = add_model(ply_file);
		if (!graph) {
			g_log.add("[ERROR] Failed to load: " + ply_file);
			return;
		}
		graph->set_name(name);
		if (auto *graph_vertices = graph->renderer()->get_points_drawable("vertices", false)) {
			const easy3d::Texture *tex = easy3d::TextureManager::request(ROOT_DIR "/resources/colormap.png", 42);
			try {
				graph_vertices->set_scalar_coloring(easy3d::State::VERTEX, "v:instance", tex, 0.0f, 0.0f);
			} catch (const std::exception &e) {
				g_log.add(std::string("[WARN] Scalar coloring failed: ") + e.what());
			}
		}
		center_view();
	}


	// Reset pipeline state only; keeps the current cfg and GUI buffers intact so the user's
	// in-flight edits aren't blown away. cfg_dirty_ is set so the next run forces a Confirm,
	// which wipes leftover output (.iterations, intermediate PLYs) from the previous run.
	void reset_pipeline() {
		completed_ = 0;
		clear_error();
		original_cloud_ = {};
		input_cloud_ = {};
		lap_skeleton_ = {};
		fps_points_ = {};
		lop_points_ = {};
		graph_obj_.reset();
		initial_graph_ = {};
		mst_graph_ = {};
		pruned_mst_ = {};
		skel_sem_.clear();
		skel_inst_.clear();
		pred_sem_.clear();
		pred_inst_.clear();
		while (!models().empty()) {
			delete_model(models().back().get());
		}
		cfg_dirty_ = true;

		g_log.add("[INFO] Pipeline reset.");
	}
};


// Entry point
int main(int argc, char **argv) {
	easy3d::initialize();
	MyViewer viewer;
	return viewer.run();
}
