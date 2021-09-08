#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	//TODO LookAt direction and Up Direction
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.

	float radian = rotation_angle / 180 * MY_PI;

	model << cos(radian), -sin(radian), 0, 0,
		sin(radian), cos(radian), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return model;
}

// Use Rodrigues' Rotation Formula to get rotation matrix
Eigen::Matrix4f get_axis_rotation(Vector3f axis, float angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	float radian = angle / 180 * MY_PI;
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f tmp = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Rodrigues;
	
	// Rodrigues' Rotation Formula
	tmp << 0, -axis[2], axis[1], axis[2], 0, -axis[0], -axis[1], axis[0], 0;
	Rodrigues = cos(radian) * I + (1 - cos(radian)) * axis * axis.adjoint() + sin(radian) * tmp;
	model << Rodrigues(0, 0), Rodrigues(0, 1), Rodrigues(0, 2), 0,
		Rodrigues(1, 0), Rodrigues(1, 1), Rodrigues(1, 2), 0,
		Rodrigues(2, 0), Rodrigues(2, 1), Rodrigues(2, 2), 0,
		0, 0, 0, 1;
	return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	// Students will implement this function

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.

	Eigen::Matrix4f ortho_trans = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Identity();

	// n and f are coordinate ,zNear and zFar are distance
	float r, l, t, b, n = -zNear, f = -zFar;
	float radian_fov = eye_fov * MY_PI / 180;
	t = tan(radian_fov / 2) * zNear;
	b = -t;
	r = t * aspect_ratio;
	l = -r;

	ortho_trans << 1, 0, 0, -(r + l) / 2,
		0, 1, 0, -(t + b) / 2,
		0, 0, 1, -(n + f) / 2,
		0, 0, 0, 1;

	ortho_scale << 2 / (r - l), 0, 0, 0,
		0, 2 / (t - b), 0, 0,
		0, 0, 2 / (n - f), 0,
		0, 0, 0, 1;

	persp2ortho << n, 0, 0, 0,
		0, n, 0, 0,
		0, 0, n + f, -n * f,
		0, 0, 1, 0;
	projection = ortho_scale * ortho_trans * persp2ortho;

	return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};


	std::vector<Eigen::Vector3f> pos
	{
			{2, 0, -2},
			{0, 2, -2},
			{-2, 0, -2},
			{3.5, -1, -5},
			{2.5, 1.5, -5},
			{-1, 0.5, -5}
	};

	std::vector<Eigen::Vector3i> ind
	{
			{0, 1, 2},
			{3, 4, 5}
	};

	std::vector<Eigen::Vector3f> cols
	{
			{217.0, 238.0, 185.0},
			{217.0, 238.0, 185.0},
			{217.0, 238.0, 185.0},
			{185.0, 217.0, 238.0},
			{185.0, 217.0, 238.0},
			{185.0, 217.0, 238.0}
	};

    
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
	auto col_id = r.load_colors(cols);


    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
		r.set_model(get_axis_rotation({ 1,0,0 }, angle));
		r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

	while (key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		//r.set_model(get_model_matrix(angle));
		r.set_model(get_axis_rotation({ 0,0,1 }, angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 1, 10));

		r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		const float stride = 0.1f;
		if (key == 'a') {
			eye_pos[0] -= stride;
		}
		else if (key == 'd') {
			eye_pos[0] += stride;
		}
		else if (key == 'w') {
			eye_pos[2] -= stride;
		}
		else if (key == 's') {
			eye_pos[2] += stride;
		}
		else if (key == 'q') {
			eye_pos[1] -= stride;
		}
		else if (key == 'e') {
			eye_pos[1] += stride;
		}
	}

    return 0;
}
