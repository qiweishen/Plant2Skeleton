#include "viewer.h"
#include <easy3d/core/point_cloud.h>
#include <easy3d/util/initializer.h>
#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/backends/imgui_impl_opengl3.h>
#include <iostream>



class MyViewer : public easy3d::ViewerImGui {
public:
    MyViewer() : ViewerImGui("SkelSeg") {}

protected:
    void post_draw() override {
        // About dialog
        if (show_about_) {
        	ImGui::SetNextWindowPos(ImVec2(static_cast<float>(width()) * 0.5f, static_cast<float>(height()) * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Begin("About", &show_about_, ImGuiWindowFlags_NoResize);
            ImGui::Text("SkelSeg");
            ImGui::End();
        }

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
        if (ImGui::BeginMainMenuBar()) {
            draw_menu_file();
            draw_menu_view();

            // Your custom menu
            if (ImGui::BeginMenu("Reconstruction")) {
                if (ImGui::MenuItem("Reconstruct Skeleton")) {
                    reconstruct_skeleton();
                }
                if (ImGui::MenuItem("Add Leaves")) {
                    add_leaves();
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Help")) {
                ImGui::MenuItem("About", nullptr, &show_about_);
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
    bool show_about_ = false;

    void reconstruct_skeleton() {
        auto* cloud = dynamic_cast<easy3d::PointCloud*>(current_model());
        if (cloud) {
            std::cout << "Reconstructing skeleton from "
                      << cloud->n_vertices() << " points\n";
        }
    }

    void add_leaves() {
        std::cout << "Adding leaves...\n";
    }
};

int main(int argc, char** argv) {
    easy3d::initialize();
    MyViewer viewer;
    return viewer.run();
}