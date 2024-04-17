#pragma once

#include "GUI.hpp"
#include "Eigen/Eigen"

namespace TDA {

inline Eigen::Vector3f HSV2RGB(double H, double S, double V) {
    // std::cerr << H;
    // assert(0 <= H && H <= 1);
    double C = V * S;
    double X = C * (1 - std::abs(std::fmod((6 * H), 2.0) - 1));
    double m = V - C;
    Eigen::Vector3f RGB;
    if (H < 1.0 / 6) {
        RGB << C, X, 0;
    } else if (H < 2.0 / 6) {
        RGB << X, C, 0;
    } else if (H < 3.0 / 6) {
        RGB << 0, C, X;
    } else if (H < 4.0 / 6) {
        RGB << 0, X, C;
    } else if (H < 5.0 / 6) {
        RGB << X, 0, C;
    } else {
        RGB << C, 0, X;
    }
    
    return RGB.array() + m;
}

class PointCloud {
public:
    void SetCoordinates(const Renderer::RowMajorMatrixX3f& coords);
    void SetColors(const Renderer::RowMajorMatrixX3f& colors);
    void SetEdges(const Renderer::RowMajorMatrixX2u& edges);

    Renderer::VertexArray* _va;
    int _num_edges = 0;
    int _edge_capacity = 0;

    static PointCloud* GetPointCloud (
        int num_vertices,
        int num_edges
    );

protected:
    PointCloud(Renderer::VertexArray* va): _va(va) {}

};

void RenderPoint(const Camera& camera, const Eigen::Vector3f& point, const Eigen::Vector3f& color);
void RenderPointCloud(const Renderer::Shader& shader, const Camera& camera, const PointCloud& point_cloud);
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
	void SetThreshold(float threshold);
	void SetVertices(const Renderer::RowMajorMatrixX3f& vertices);

	void DisplayFunc() override;
	void MouseButtonCallback(GLFWwindow *window, int button, int action, int mods) override;
	void MouseMotionCallback(GLFWwindow *window, double x, double y) override;
	void KeyboardCallback(GLFWwindow *window, int key, int scancode, int action, int mods) override;
	void WindowSizeCallback(GLFWwindow *window, int width, int height) override;

    void RenderImGuiPanel();

    void ClearColor();
	void ComputeGlobal();
	void ComputeLocal();
	void ComputeGlobalGraph();
	void ComputeLocalGraph();
    void ComputeSelectedVertices();

protected:
	bool _left_down = false, _right_down = false, _middle_down = false;
	bool _left_alt_down = false;
    double _last_x, _last_y;

    float _mouse_movement_sensitivity = 0.01f;
    float _mouse_rotation_sensitivity = 0.1f;
    float _mouse_zoom_sensitivity = 0.1f;

	Camera* _camera = nullptr;
	Renderer::Shader* _point_cloud_shader = nullptr;
	Renderer::Shader* _wireframe_shader = nullptr;

    static const int kCenterSelectMode = 0;
    static const int kGlobalWireframeMode = 1;
    static const int kLocalWireframeMode = 2;
    static const int kColorMode = 3;

    int _cur_mode = 0;

	Renderer::RowMajorMatrixX3f _vertices;
	Renderer::RowMajorMatrixX3f _base_colors;
    PointCloud* _point_cloud = nullptr;
	float _threshold = 0;
	float _last_global_threshold = 0;
	float _last_local_threshold = 0;

    Eigen::Vector3f _center;
    Eigen::Vector3f _last_center = Eigen::Vector3f::Zero();
    float _radius;
    float _last_radius = 0;

};


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
	std::vector<unsigned int> _vertex_ids;

	float _threshold = 0;
	
	std::vector<unsigned int> _center_ids;
    Eigen::VectorXd _center;
    Eigen::VectorXd _last_center;
    float _radius;
    float _last_radius = 0;

};

}
