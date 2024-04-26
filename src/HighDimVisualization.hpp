#pragma once

#include "GUI.hpp"
#include "Eigen/Eigen"
#include "Visualization.hpp"
#include "CircularCoordinate.hpp"
#include "DimensionReduction.hpp"
#include "Render.hpp"
#include "TDAConfig.hpp"
#include "imgui.h"
#include "imgui_impl_opengl3.h"

#include <filesystem>

namespace fs = std::filesystem;

namespace TDA {

template<int dim>
class HighDimGUI : public Renderer::GUI {
public:
	// call it before MainLoop
	void PreCompute();

    void SetCamera(
		const glm::vec3& position,
		const float rotz,
		const float roty,
        const float near,
        const float far
    );
	void SetVertices(const Eigen::MatrixXd& vertices);
	

	void DisplayFunc() override;
	void MouseButtonCallback(GLFWwindow *window, int button, int action, int mods) override;
	void MouseMotionCallback(GLFWwindow *window, double x, double y) override;
	void KeyboardCallback(GLFWwindow *window, int key, int scancode, int action, int mods) override;
	void WindowSizeCallback(GLFWwindow *window, int width, int height) override;

	void RenderSelectBox();
    void RenderImGuiPanel();

    void ClearAll();
    void ComputeCenter();
	void ComputeLocalVertices();
	void ComputeGraph();
	void ComputeCircularCoords();

protected:
	bool _left_down = false, _right_down = false, _middle_down = false;
	bool _left_alt_down = false;
	bool _left_ctrl_down = false;
    double _last_x, _last_y;
	double _select_begin_x, _select_begin_y;

    float _mouse_movement_sensitivity = 0.01f;
    float _mouse_rotation_sensitivity = 0.1f;
    float _mouse_zoom_sensitivity = 0.1f;

	Camera* _camera = nullptr;
	Renderer::Shader* _point_cloud_shader = nullptr;
	Renderer::Shader* _wireframe_shader = nullptr;
	Renderer::Shader* _backbuffer_shader = nullptr;

	enum class HighDimGUIMode {
		kCenterSelectMode = 0,
		kRadiusSelectMode = 1,
		kWireframeMode = 2,
		kColorMode = 3
	} _mode;

	int _num_vertices;
	Eigen::MatrixXd _vertices;

	Renderer::RowMajorMatrixX3f _vertices_3d;
	Renderer::RowMajorMatrixX3f _base_colors;

    PointCloud* _point_cloud = nullptr;
	GLuint _vertex_backbuffer = 0;
	GLuint _vertex_backbuffer_color;
	GLuint _vertex_backbuffer_depth;
	std::vector<unsigned int> _backbuffer_data;
	Renderer::VertexArray* _vertex_select_array = nullptr;

	float _threshold = 0;
	
	std::vector<unsigned int> _center_ids;
    Eigen::VectorXd _center;
	Eigen::Vector3f _center_3d;
    Eigen::VectorXd _last_center;
    float _radius;
    float _last_radius = 0;

};

template<int dim>
void HighDimGUI<dim>::PreCompute() {
	_point_cloud_shader = new Renderer::Shader(
		fs::path(TDA_SHADER_PATH) / "colored-point-cloud.vert",
		fs::path(TDA_SHADER_PATH) / "colored-point-cloud.frag"
	);
    _wireframe_shader = new Renderer::Shader(
	    fs::path(TDA_SHADER_PATH) / "wireframe.vert",
		fs::path(TDA_SHADER_PATH) / "wireframe.frag"
    );
	_backbuffer_shader = new Renderer::Shader(
		fs::path(SHADER_DIR) / "vertex-select.vert",
		fs::path(SHADER_DIR) / "vertex-select.frag"
	);

	glGenFramebuffers(1, &_vertex_backbuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, _vertex_backbuffer);

	glGenTextures(1, &_vertex_backbuffer_color);
	glBindTexture(GL_TEXTURE_2D, _vertex_backbuffer_color);
	glTexImage2D(
		GL_TEXTURE_2D,
		0,
		GL_R32UI,
		_width,
		_height,
		0,
		GL_RED_INTEGER,
		GL_UNSIGNED_INT,
		nullptr
	);
	glTexParameteri(
		GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST
	);
	glTexParameteri(
		GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST
	);
	glBindTexture(GL_TEXTURE_2D, 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _vertex_backbuffer_color, 0);

	glGenRenderbuffers(1, &_vertex_backbuffer_depth);
	glBindRenderbuffer(GL_RENDERBUFFER, _vertex_backbuffer_depth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, _width, _height);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _vertex_backbuffer_depth);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		spdlog::error("Unable to initialize back buffer, error code: {}", glCheckFramebufferStatus(GL_FRAMEBUFFER));
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);
	
	_backbuffer_data.resize(mode->width * mode->height);
}

template<int dim>
void HighDimGUI<dim>::SetVertices(const Eigen::MatrixXd &vertices) {
	_num_vertices = vertices.rows();

	_vertices = vertices;
	PCA<3> pca;
	pca.Fit(vertices);
	_vertices_3d = pca.Transform(vertices).cast<float>();

	delete _point_cloud;
	_point_cloud = TDA::PointCloud::GetPointCloud(_num_vertices, 0);
	_point_cloud->SetCoordinates(_vertices_3d);
    _base_colors = Renderer::RowMajorMatrixX3f::Ones(
        vertices.rows(), 3
    );
    _point_cloud->SetColors(_base_colors);

	std::vector<unsigned int> vertex_ids;
	vertex_ids.reserve(_num_vertices);
	for (unsigned int i = 0; i < _num_vertices; i++) {
		vertex_ids.push_back(i);
	}

	delete _vertex_select_array;
	_vertex_select_array = new Renderer::VertexArray(_num_vertices, 0);
	_vertex_select_array->ShallowCopyArrayBufferWithConfig(
		Renderer::GLUtility::VERTEX_LOCATION,
		*_point_cloud->_va,
		Renderer::GLUtility::VERTEX_LOCATION
	);

	auto vid_buffer = Renderer::GLUtility::CreateArrayBuffer(
        sizeof(unsigned int) * _num_vertices,
        GL_STATIC_DRAW
    );

    _vertex_select_array->AddArrayBuffer(
		Renderer::GLUtility::OTHER_LOCATION0,
		vid_buffer
	);

    _vertex_select_array->ConfigArrayBufferI(
        Renderer::GLUtility::OTHER_LOCATION0,
        1,
        GL_UNSIGNED_INT,
        sizeof(unsigned int),
        0
    );

	_vertex_select_array->FillArrayBuffer(
		Renderer::GLUtility::OTHER_LOCATION0,
		vertex_ids.data(),
		sizeof(unsigned int) * _num_vertices
	);
}

template<int dim>
void HighDimGUI<dim>::SetCamera(
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

template<int dim>
void HighDimGUI<dim>::DisplayFunc() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

    switch (_mode) {
        case HighDimGUIMode::kCenterSelectMode: {
            RenderPointCloud(
                *_point_cloud_shader, 
                *_camera,
                *_point_cloud
            );
			if (!_center_ids.empty()) {
				RenderPoint(
					*_camera,
					_center_3d,
					(Eigen::Vector3f() << 0, 0, 1.0).finished()
				);
			}
			
            glBindFramebuffer(GL_FRAMEBUFFER, _vertex_backbuffer);
			GLuint val = _num_vertices;
			glClearBufferuiv(GL_COLOR, 0, &val);
			glClear(GL_DEPTH_BUFFER_BIT);
            Renderer::RenderUtility::RenderToVertexBackBuffer(
                *_backbuffer_shader,
                *_camera,
				*_vertex_select_array
            );
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            if (_left_down && _left_ctrl_down && !_left_alt_down) {
                RenderSelectBox();
            }
            break;
		}
        case HighDimGUIMode::kRadiusSelectMode: {
			ComputeLocalVertices();
            RenderPointCloud(
                *_point_cloud_shader,
                *_camera,
                *_point_cloud
            );
            break;
		}
        case HighDimGUIMode::kWireframeMode: {
			if (_radius != _last_radius) {
				_last_radius = _radius;
            	ComputeGraph();
			}
            RenderEdges(
                *_wireframe_shader,
                *_camera,
                *_point_cloud
            );
            break;
		}
        case HighDimGUIMode::kColorMode: {
            RenderPointCloud(
                *_point_cloud_shader,
                *_camera,
                *_point_cloud
            );
            break;
		}
    };

    RenderImGuiPanel();

	glfwSwapBuffers(_window);
}

template<int dim>
void HighDimGUI<dim>::RenderSelectBox() {
	float min_x = Normalize(std::min(_select_begin_x, _last_x), _width);
	float max_x = Normalize(std::max(_select_begin_x, _last_x), _width);
	float min_y = Normalize(std::min(_height - _select_begin_y, _height - _last_y), _height);
	float max_y = Normalize(std::max(_height - _select_begin_y, _height - _last_y), _height);

    glUseProgram(0);
    static const float default_color[] = {1, 1, 1};
    static const float color[] = {0.2, 0.3, 1, 0.4};
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glColor4fv(color);

    glBegin(GL_TRIANGLE_STRIP);
    glVertex2f(min_x, min_y);
    glVertex2f(min_x, max_y);
    glVertex2f(max_x, min_y);
    glVertex2f(max_x, max_y);
    glEnd();

    glColor3fv(default_color);
    glPointSize(1);
}

template<int dim>
void HighDimGUI<dim>::RenderImGuiPanel() {
    ImGui::Begin("Control Panel");

    static const char* modes[] = {
        "center select",
        "radius select",
        "wireframe",
        "color"
    };

    const char* combo_preview_value = modes[static_cast<int>(_mode)];
    if (ImGui::BeginCombo("Mode", combo_preview_value))
    {
        for (int n = 0; n < IM_ARRAYSIZE(modes); n++)
        {
            const bool is_selected = (static_cast<int>(_mode) == n);
            if (ImGui::Selectable(modes[n], is_selected)) {
                _mode = HighDimGUIMode(n);
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::Button("Clear All")) {
        ClearAll();
    }
	if (ImGui::Button("Compute Circular Coordinates")) {
		ComputeCircularCoords();
	}

    ImGui::End();

	switch (_mode) {
		case HighDimGUIMode::kCenterSelectMode:
			ImGui::Begin("Center Seclect");
			ImGui::Text("Number of centers selected: %d", int(_center_ids.size()));
			ImGui::End();
			break;
		case HighDimGUIMode::kRadiusSelectMode:
			ImGui::Begin("Radius Select");
			ImGui::SliderFloat("radius", &_radius, 0, 3);
			ImGui::End();
			break;
		case HighDimGUIMode::kWireframeMode:
			ImGui::Begin("Threshold Select");
			ImGui::SliderFloat("threshold", &_threshold, 0, 2.5);
			ImGui::End();
			break;
		default:
			break;
	}

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

template<int dim>
void HighDimGUI<dim>::ClearAll() {
    _point_cloud->SetColors(_base_colors);
	_center_ids.clear();
	_threshold = 0;
	_last_radius = 0;
}

template<int dim>
void HighDimGUI<dim>::ComputeCenter() {
	GLint min_x = std::round(std::min(_select_begin_x, _last_x));
	GLint max_x = std::round(std::max(_select_begin_x, _last_x));
	GLint min_y = std::round(std::min(_height - _select_begin_y, _height - _last_y));
	GLint max_y = std::round(std::max(_height - _select_begin_y, _height - _last_y));
	min_x = std::max(0, min_x);
	min_y = std::max(0, min_y);
	max_x = std::min(_width, max_x);
	max_y = std::min(_height, max_y);
	auto width = max_x - min_x;
	auto height = max_y - min_y;

	glBindFramebuffer(GL_FRAMEBUFFER, _vertex_backbuffer);

	glReadPixels(
		min_x, min_y,
		width, height,
		GL_RED_INTEGER,
		GL_UNSIGNED_INT,
		_backbuffer_data.data()
	);

	_center_ids.clear();
	for (int i = 0; i < width * height; i++) {
		if (_backbuffer_data[i] < _num_vertices) {
			_center_ids.push_back(_backbuffer_data[i]);
		}
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	if (!_center_ids.empty()) {
		_center = Eigen::VectorXd::Zero(_vertices.cols());
		_center_3d = Eigen::Vector3f::Zero();
		for (const auto center_id : _center_ids) {
			_center += _vertices.row(center_id);
			_center_3d += _vertices_3d.row(center_id);
		}
		_center /= _center_ids.size();
		_center_3d /= _center_ids.size();
	} else {
		spdlog::warn("No center selected");
	}
}

template<int dim>
void HighDimGUI<dim>::ComputeLocalVertices() {
	Renderer::RowMajorMatrixX3f colors = _base_colors;
	Eigen::VectorXd distances = (_vertices.rowwise() - _center.transpose()).rowwise().norm();
	for (int i = 0; i < _num_vertices; i++) {
		if (distances(i) < _radius) {
			colors.row(i) << 1, 0, 0;
		}
	}
	_point_cloud->SetColors(colors);
}

template<int dim>
void HighDimGUI<dim>::ComputeGraph() {
	std::vector<std::pair<unsigned int, unsigned int>> edges;

	const int num_vertices = _vertices.rows();
    for (int i = 0; i < num_vertices; i++) {
        if ((_vertices.row(i) - _center.transpose()).norm() < _radius) {
            for (int j = i + 1; j < num_vertices; j++) {
                if ((_vertices.row(i) - _vertices.row(j)).norm() < _threshold) {
                    edges.push_back(std::make_pair(i, j));
                }
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

template<int dim>
void HighDimGUI<dim>::ComputeCircularCoords() {
    auto result = TDA::CalculateLocalCircularCoordinate<dim>(
        _vertices,
        _center,
        _radius,
        _threshold
    );

    if (result.has_value()) {
	    const int num_vertices = _vertices.rows();
        Renderer::RowMajorMatrixX3f colors(num_vertices, 3);
        auto& [local_vertex_ids, circular_coords] = result.value();
        auto& hues = circular_coords[0];
        int local_id = 0;
        for (int i = 0; i < num_vertices; i++) {
            if (i == local_vertex_ids[local_id]) {
                colors.row(i) = TDA::HSV2RGB(
                    hues[local_id + 1] - std::floor(hues[local_id + 1]),
                    1.0, 1.0
                );
                local_id++;
            } else {
                colors.row(i) << 1, 1, 1;
            }
        }
        _point_cloud->SetColors(colors);
    } else {
		spdlog::warn("No valid circular coordinates");
        _point_cloud->SetColors(_base_colors);
    }
}

#define PRESS(X) (button == GLFW_MOUSE_BUTTON_##X && action == GLFW_PRESS)
#define RELEASE(X) (button == GLFW_MOUSE_BUTTON_##X && action == GLFW_RELEASE)

template<int dim>
void HighDimGUI<dim>::MouseButtonCallback(
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
    if (_mode == HighDimGUIMode::kCenterSelectMode && _left_ctrl_down) {
        if (PRESS(LEFT)) {
            glfwGetCursorPos(window, &_select_begin_x, &_select_begin_y);
        }
        if (RELEASE(LEFT)) {
            ComputeCenter();
        }
    }

    if (action == GLFW_PRESS) {
        glfwGetCursorPos(window, &_last_x, &_last_y);
    }
}
#undef PRESS
#undef RELEASE

template<int dim>
void HighDimGUI<dim>::MouseMotionCallback(
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

template<int dim>
void HighDimGUI<dim>::KeyboardCallback(
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

	if (PRESS(LEFT_CONTROL)) {
		_left_ctrl_down = true;
	}

	if (RELEASE(LEFT_CONTROL)) {
		_left_ctrl_down = false;
	}

}
#undef PRESS
#undef RELEASE

template<int dim>
void HighDimGUI<dim>::WindowSizeCallback(GLFWwindow *window, int width, int height) {
	glViewport(0, 0, width, height);
	_width = width;
	_height = height;
	_camera->SetAspectRatio((float) width / height);
}

}