#include "Visualization.hpp"
#include "CircularCoordinate.hpp"
#include <filesystem>
#include "DimensionReduction.hpp"
#include "Render.hpp"
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
        GL_DYNAMIC_DRAW
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

void RenderPoint(
    const Camera& camera,
    const Eigen::Vector3f &point,
    const Eigen::Vector3f &color
) {
    glUseProgram(0);
    static const float default_color[] = {1, 1, 1};
    glm::vec4 point_4d(point(0), point(1), point(2), 1.0);
    auto transformed_point = camera.GetProjectionMatrix() * camera.GetViewMatrix() * point_4d;
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glPointSize(5);
    glColor3f(color(0), color(1), color(2));

    glBegin(GL_POINTS);
    glVertex4f(
        transformed_point[0],
        transformed_point[1],
        transformed_point[2],
        transformed_point[3]
    );
    glEnd();

    glColor3fv(default_color);
    glPointSize(1);
}

void RenderPointCloud(
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
    _base_colors = Renderer::RowMajorMatrixX3f::Ones(
        vertices.rows(), vertices.cols()
    );
    _point_cloud->SetColors(_base_colors);
}

void TDAGUI::DisplayFunc() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

    switch (_cur_mode) {
        case kCenterSelectMode:
            ComputeSelectedVertices();
            RenderPointCloud(
                *_point_cloud_shader,
                *_camera,
                *_point_cloud
            );
            RenderPoint(
                *_camera,
                _center,
                (Eigen::Vector3f() << 0, 0, 1).finished()
            );
            break;
        case kLocalWireframeMode:
            if (_threshold != _last_local_threshold || _radius != _last_radius || _center != _last_center) {
                _last_global_threshold = _threshold;
                _last_radius = _radius;
                _last_center = _center;
                ComputeLocalGraph();
            }
            RenderEdges(*_wireframe_shader, *_camera, *_point_cloud);
            break;
        case kGlobalWireframeMode:
            if (_threshold != _last_global_threshold) {
                _last_global_threshold = _threshold;
                ComputeGlobalGraph();
            }
            RenderEdges(*_wireframe_shader, *_camera, *_point_cloud);
            break;
        case kColorMode:
            RenderPointCloud(*_point_cloud_shader, *_camera, *_point_cloud);
            break;
    };

    RenderImGuiPanel();

	glfwSwapBuffers(_window);
}

void TDAGUI::RenderImGuiPanel() {
    ImGui::Begin("Control Panel");

    static const char* modes[] = {
        "center select",
        "global wireframe",
        "local wireframe",
        "color"
    };

    const char* combo_preview_value = modes[_cur_mode];
    if (ImGui::BeginCombo("Mode", combo_preview_value))
    {
        for (int n = 0; n < IM_ARRAYSIZE(modes); n++)
        {
            const bool is_selected = (_cur_mode == n);
            if (ImGui::Selectable(modes[n], is_selected)) {
                _cur_mode = n;
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::Button("Clear Color")) {
        ClearColor();
    }

    ImGui::End();

    ImGui::Begin("Global Circular Coordinate");

	ImGui::SliderFloat("threshold", &_threshold, 0, 2.5);
	if (ImGui::Button("Compute Global")) {
		ComputeGlobal();
	}

    ImGui::End();

    ImGui::Begin("Local Circular Coordinate");

	ImGui::SliderFloat("threshold", &_threshold, 0, 2.5);
    ImGui::SliderFloat("center[0]", &_center(0), -1, 1);
    ImGui::SliderFloat("center[1]", &_center(1), -1, 1);
    ImGui::SliderFloat("center[2]", &_center(2), -1, 1);
    ImGui::SliderFloat("radius", &_radius, 0, 3);
    if (ImGui::Button("Compute Local")) {
        ComputeLocal();
    }

    ImGui::End();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void TDAGUI::ClearColor() {
    _point_cloud->SetColors(_base_colors);
}

void TDAGUI::ComputeSelectedVertices() {
    const int num_vertices = _vertices.rows();
    Renderer::RowMajorMatrixX3f colors(num_vertices, 3);
    Eigen::VectorXf distance = (_vertices.rowwise() - _center.transpose()).rowwise().norm();
    for (int i = 0; i < num_vertices; i++) {
        if (distance(i) < _radius + _threshold) {
            colors.row(i) << 1, 0, 0;
        } else {
            colors.row(i) << 1, 1, 1;
        }
    }
    _point_cloud->SetColors(colors);
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
	    _point_cloud->SetColors(colors);
    } else {
        _point_cloud->SetColors(_base_colors);
    }
}

void TDAGUI::ComputeLocal() {
    auto result = TDA::CalculateLocalCircularCoordinate<3>(
        _vertices.cast<double>(),
        _center.cast<double>(),
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
        _point_cloud->SetColors(_base_colors);
    }
    
}

void TDAGUI::ComputeGlobalGraph() {
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

void TDAGUI::ComputeLocalGraph() {
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

void HighDimGUI::PreCompute() {
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
}

void HighDimGUI::SetVertices(const Eigen::MatrixXd &vertices) {
	_num_vertices = vertices.rows();

	_vertices = vertices;
	PCA<3> pca;
	pca.Fit(vertices);
	_vertices_3d = pca.Transform(vertices);

	delete _point_cloud;
	_point_cloud = TDA::PointCloud::GetPointCloud(_num_vertices, 0);
	_point_cloud->SetCoordinates(_vertices_3d);
    _base_colors = Renderer::RowMajorMatrixX3f::Ones(
        vertices.rows(), 3
    );
    _point_cloud->SetColors(_base_colors);

	_vertex_ids.reserve(_num_vertices);
	for (unsigned int i = 0; i < _num_vertices; i++) {
		_vertex_ids.push_back(i);
	}

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
		_vertex_ids.data(),
		sizeof(unsigned int) * _num_vertices
	);
}

void HighDimGUI::SetCamera(
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

void HighDimGUI::DisplayFunc() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

    switch (_mode) {
        case HighDimGUIMode::kCenterSelectMode:
			// TODO:
            RenderPointCloud(
                *_point_cloud_shader, 
                *_camera,
                *_point_cloud
            );
            glBindFramebuffer(GL_FRAMEBUFFER, _vertex_backbuffer);
            Renderer::RenderUtility::RenderToVertexBackBuffer(
                *_backbuffer_shader,
                *_camera,
                *_point_cloud->_va
            );
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            if (_left_down && !_left_alt_down) {
                RenderSelectBox();
            }
            break;
        case HighDimGUIMode::kRadiusSelectMode:
			ComputeLocalVertices();
            RenderPointCloud(
                *_point_cloud_shader,
                *_camera,
                *_point_cloud
            );
            break;
        case HighDimGUIMode::kWireframeMode:
            ComputeGraph();
            RenderEdges(
                *_wireframe_shader,
                *_camera,
                *_point_cloud
            );
            break;
        case HighDimGUIMode::kColorMode:
            RenderPointCloud(
                *_point_cloud_shader,
                *_camera,
                *_point_cloud
            );
            break;
    };

    RenderImGuiPanel();

	glfwSwapBuffers(_window);
}

void HighDimGUI::RenderSelectBox() {
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

void HighDimGUI::RenderImGuiPanel() {
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
			ImGui::LabelText("Number of centers selected", "%d", int(_center_ids.size()));
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

void HighDimGUI::ClearAll() {
    _point_cloud->SetColors(_base_colors);
	_center_ids.clear();
	_threshold = 0;
	_last_radius = 0;
}

void HighDimGUI::ComputeCenter() {
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

	std::vector<unsigned int> selected_vertex_ids;
	for (int i = 0; i < width * height; i++) {
		if (_backbuffer_data[i] < _num_vertices) {
			selected_vertex_ids.push_back(_backbuffer_data[i]);
		}
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	if (!selected_vertex_ids.empty()) {
		_center = Eigen::VectorXd::Zero(_num_vertices);
		for (const auto selected_vertex_id : selected_vertex_ids) {
			_center += _vertices.row(selected_vertex_id);
		}
		_center /= selected_vertex_ids.size();
	}
}

void HighDimGUI::ComputeLocalVertices() {
	Renderer::RowMajorMatrixX3f colors = _base_colors;
	Eigen::VectorXd distances = (_vertices.rowwise() - _center.transpose()).rowwise().norm();
	for (int i = 0; i < _num_vertices; i++) {
		if (distances(i) < _radius) {
			colors.row(i) << 1, 0, 0;
		}
	}
	_point_cloud->SetColors(colors);
}

void HighDimGUI::ComputeGraph() {
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

void HighDimGUI::ComputeCircularCoords() {
    auto result = TDA::CalculateLocalCircularCoordinate<Eigen::Dynamic>(
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
void HighDimGUI::MouseButtonCallback(
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
    if (_mode == HighDimGUIMode::kCenterSelectMode) {
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

void HighDimGUI::MouseMotionCallback(
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
void HighDimGUI::KeyboardCallback(
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

void HighDimGUI::WindowSizeCallback(GLFWwindow *window, int width, int height) {
	glViewport(0, 0, width, height);
	_width = width;
	_height = height;
	_camera->SetAspectRatio((float) width / height);
	_backbuffer_data.resize(width * height);
}


}