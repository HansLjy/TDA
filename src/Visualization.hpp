#pragma once

#include "GUI.hpp"
#include "Eigen/Eigen"

namespace TDA {


class PointCloud {
public:
    void SetCoordinates(const Renderer::RowMajorMatrixX3f& coords);
    void SetColors(const Renderer::RowMajorMatrixX3f& colors);
    void SetEdges(const Renderer::RowMajorMatrixX2u& edges);

    Renderer::VertexArray* _va;
    int _num_edges;
    int _edge_capacity;

    static PointCloud* GetPointCloud (
        int num_vertices,
        int num_edges
    );

protected:
    PointCloud(Renderer::VertexArray* va): _va(va) {}

};

void RenderColoredPointCloud(const Renderer::Shader& shader, const Camera& camera, const PointCloud& point_cloud);
void RenderEdges(const Renderer::Shader& shader, const Camera& camera, const PointCloud& point_cloud);


class TDAGUI : public Renderer::GUI {
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

    void SetPointCloud(
        PointCloud* point_cloud
    );

	void DisplayFunc() override;
	void MouseButtonCallback(GLFWwindow *window, int button, int action, int mods) override;
	void MouseMotionCallback(GLFWwindow *window, double x, double y) override;
	void KeyboardCallback(GLFWwindow *window, int key, int scancode, int action, int mods) override;
	void WindowSizeCallback(GLFWwindow *window, int width, int height) override;


protected:
	bool _left_down = false, _right_down = false, _middle_down = false;
	bool _left_alt_down = false;
    double _last_x, _last_y;

    float _mouse_movement_sensitivity = 0.1f;
    float _mouse_rotation_sensitivity = 0.1f;
    float _mouse_zoom_sensitivity = 0.1f;

	Camera* _camera = nullptr;
	Renderer::Shader* _point_cloud_shader = nullptr;
	Renderer::Shader* _wireframe_shader = nullptr;

	bool _wire_frame_mode = false;

    PointCloud* _point_cloud;
};


}
