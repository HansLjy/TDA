#include "Visualization.hpp"
#include "CircularCoordinate.hpp"
#include <filesystem>
#include "TDAConfig.hpp"
#include "imgui.h"
#include "imgui_impl_opengl3.h"

namespace fs = std::filesystem;

namespace TDA {

PointCloud* PointCloud::GetPointCloud(
    int num_vertices,
    int num_edges
) {
    auto va = new Renderer::VertexArray(num_vertices, 0);
    
    auto vertex_buffer = Renderer::GLUtility::CreateArrayBuffer(
        num_vertices * 3 * sizeof(float),
        GL_STATIC_DRAW
    );
    va->AddArrayBuffer(Renderer::GLUtility::VERTEX_LOCATION, vertex_buffer);
    va->ConfigArrayBuffer(
        Renderer::GLUtility::VERTEX_LOCATION,
        3,
        GL_FLOAT,
        false,
        3 * sizeof(float),
        0
    );

    auto color_buffer = Renderer::GLUtility::CreateArrayBuffer(
        num_vertices * 3 * sizeof(float), 
        GL_STATIC_DRAW
    );
    va->AddArrayBuffer(Renderer::GLUtility::PER_VERTEX_COLOR_LOCATION, color_buffer);
    va->ConfigArrayBuffer(
        Renderer::GLUtility::PER_VERTEX_COLOR_LOCATION,
        3,
        GL_FLOAT,
        false,
        3 * sizeof(float),
        0
    );

    auto element_buffer = Renderer::GLUtility::CreateElementBuffer(
        2 * num_edges * sizeof(unsigned int),
        GL_STATIC_DRAW
    );
    va->SetElementBuffer(element_buffer);

    return new PointCloud(va);
}

void PointCloud::SetCoordinates(const Renderer::RowMajorMatrixX3f &coords) {
    _va->FillArrayBufferWithEigen(Renderer::GLUtility::VERTEX_LOCATION, coords);
}

void PointCloud::SetColors(const Renderer::RowMajorMatrixX3f &colors) {
    _va->FillArrayBufferWithEigen(Renderer::GLUtility::PER_VERTEX_COLOR_LOCATION, colors);
}

void PointCloud::SetEdges(const Renderer::RowMajorMatrixX2u &edges) {
	_num_edges = edges.rows();
    if (_num_edges > _edge_capacity) {
        _edge_capacity = _num_edges * 2;
        auto element_bffer = Renderer::GLUtility::CreateElementBuffer(
            sizeof(unsigned int) * _edge_capacity * 2, GL_DYNAMIC_DRAW
        );
        _va->SetElementBuffer(element_bffer);
    }
    _va->FillElementBuffer(edges.data(), edges.size() * sizeof(unsigned int));
}

void RenderColoredPointCloud(
    const Renderer::Shader& shader,
    const Camera& camera,
    const PointCloud& point_cloud
) {
    shader.Use();
    shader["mvp"] = camera.GetProjectionMatrix() * camera.GetViewMatrix();
    point_cloud._va->DrawPoints();
}

void RenderEdges(
    const Renderer::Shader& shader,
    const Camera& camera,
    const PointCloud& point_cloud
) {
    shader.Use();
    shader["mvp"] = camera.GetProjectionMatrix() * camera.GetViewMatrix();
    point_cloud._va->DrawLineElements(point_cloud._num_edges);
}

void TDAGUI::PreCompute() {
	_point_cloud_shader = new Renderer::Shader(
		fs::path(TDA_SHADER_PATH) / "colored-point-cloud.vert",
		fs::path(TDA_SHADER_PATH) / "colored-point-cloud.frag"
	);
    _wireframe_shader = new Renderer::Shader(
	    fs::path(TDA_SHADER_PATH) / "wireframe.vert",
		fs::path(TDA_SHADER_PATH) / "wireframe.frag"
    );
}

void TDAGUI::SetThreshold(float threshold) {
	_threshold = threshold;
}

void TDAGUI::SetCamera(
    const glm::vec3& position,
    const float rotz, const float roty,
    const float near, const float far
) {
    delete _camera;
    _camera = new Camera(
        (float)_width / _height,
        position, rotz, roty, near, far
    );
}

void TDAGUI::SetVertices(const Renderer::RowMajorMatrixX3f &vertices) {
	delete _point_cloud;
	_point_cloud = TDA::PointCloud::GetPointCloud(vertices.rows(), 0);
	_vertices = vertices;
	_point_cloud->SetCoordinates(vertices);
}

void TDAGUI::DisplayFunc() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	if (_wire_frame_mode) {
		if (_threshold != _last_threshold) {
			_last_threshold = _threshold;
			ComputeGraph();
		}
        RenderEdges(*_wireframe_shader, *_camera, *_point_cloud);
	} else {
        RenderColoredPointCloud(*_point_cloud_shader, *_camera, *_point_cloud);
    }

    ImGui::Begin("Control Panel");
    ImGui::Checkbox("wireframe mode", &_wire_frame_mode);
	ImGui::SliderFloat("threshold", &_threshold, 0, 2.5);
	if (ImGui::Button("Compute Global")) {
		ComputeGlobal();
	}
    ImGui::End();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(_window);
}

void TDAGUI::ComputeGlobal() {
	auto circular_coords = TDA::CalculateGlobalCircularCoordinate<3>(
		_vertices.cast<double>(), _threshold
	);
	const int num_vertices = _vertices.rows();
	Renderer::RowMajorMatrixX3f colors(num_vertices, 3);
	if (circular_coords.has_value()) {
        auto& hues = circular_coords.value()[0];
        for (int i = 0; i < num_vertices; i++) {
            colors.row(i) = TDA::HSV2RGB(hues[i] - std::floor(hues[i]), 1.0, 1.0);
        }
    }
	_point_cloud->SetColors(colors);
}

void TDAGUI::ComputeGraph() {
	std::vector<std::pair<unsigned int, unsigned int>> edges;
	const int num_vertices = _vertices.rows();
    for (int i = 0; i < num_vertices; i++) {
        for (int j = i + 1; j < num_vertices; j++) {
            if ((_vertices.row(i) - _vertices.row(j)).norm() < _threshold) {
                edges.push_back(std::make_pair(i, j));
            }
        }
    }

    const int num_edges = edges.size();
    Renderer::RowMajorMatrixX2u edges_eigen(num_edges, 2);
    for (int i = 0; i < num_edges; i++) {
        edges_eigen.row(i) << edges[i].first, edges[i].second;
    }

	_point_cloud->SetEdges(edges_eigen);
}

// normalize val to [-1, 1]
inline float Normalize(float val, float span) {
	return val / span * 2 - 1;
}

#define PRESS(X) (button == GLFW_MOUSE_BUTTON_##X && action == GLFW_PRESS)
#define RELEASE(X) (button == GLFW_MOUSE_BUTTON_##X && action == GLFW_RELEASE)
void TDAGUI::MouseButtonCallback(
    GLFWwindow *window,
    int button,
    int action,
    int mods
) {
    switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT: {
            switch (action) {
                case GLFW_PRESS:
                    _left_down = true;
                    break;
                case GLFW_RELEASE:
                    _left_down = false;
                    break;
            }
            break;
        }
        case GLFW_MOUSE_BUTTON_RIGHT: {
            switch (action) {
                case GLFW_PRESS:
                    _right_down = true;
                    break;
                case GLFW_RELEASE:
                    _right_down = false;
                    break;
            }
            break;
        }
        case GLFW_MOUSE_BUTTON_MIDDLE: {
            switch (action) {
                case GLFW_PRESS:
                    _middle_down = true;
                    break;
                case GLFW_RELEASE:
                    _middle_down = false;
                    break;
            }
            break;
        }
    }

    if (action == GLFW_PRESS) {
        glfwGetCursorPos(window, &_last_x, &_last_y);
    }
}
#undef PRESS
#undef RELEASE

void TDAGUI::MouseMotionCallback(
    GLFWwindow *window,
    double x, double y
) {
	if (_left_alt_down) {
		if (_left_down) {
			// rotation
			_camera->CameraRotation(
				(x - _last_x) * _mouse_rotation_sensitivity,
				(y - _last_y) * _mouse_rotation_sensitivity
			);
		}

		if (_right_down) {
			// zoom
			_camera->CameraZoomIn(
				(y - _last_y) * _mouse_zoom_sensitivity
			);
		}

		if (_middle_down) {
			// movement
			_camera->CameraMovement(
				(x - _last_x) * _mouse_movement_sensitivity,
				(y - _last_y) * _mouse_movement_sensitivity
			);
		}

	}
	if (_left_down || _right_down || _middle_down) {
		_last_x = x;
		_last_y = y;
	}
}

#define PRESS(X) (key == GLFW_KEY_##X && action == GLFW_PRESS)
#define RELEASE(X) (key == GLFW_KEY_##X && action == GLFW_RELEASE)
void TDAGUI::KeyboardCallback(
    GLFWwindow *window,
    int key,
    int scancode,
    int action,
    int mods
) {
    if (PRESS(ESCAPE)) {
        glfwSetWindowShouldClose(window, true);
    }

	if (PRESS(LEFT_ALT)) {
		_left_alt_down = true;
	}

	if (RELEASE(LEFT_ALT)) {
		_left_alt_down = false;
	}

}
#undef PRESS
#undef RELEASE

void TDAGUI::WindowSizeCallback(GLFWwindow *window, int width, int height) {
	glViewport(0, 0, width, height);
	_width = width;
	_height = height;
	_camera->SetAspectRatio((float) width / height);
}

}