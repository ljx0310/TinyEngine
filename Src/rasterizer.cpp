//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>
#include <limits>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}


// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}
// get the cross product 
float sign(Vector2f p, Vector2f e1, Vector2f e2)
{
	Vector2f v1 = p - e1;
	Vector2f v2 = e2 - e1;
	return v1.x()* v2.y() - v1.y() * v2.x();
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Vector2f p = { x+0.5,y+0.5 }; //pixel's center
	Vector2f v0 = { _v[0].x(),_v[0].y() };
	Vector2f v1 = { _v[1].x(),_v[1].y() };
	Vector2f v2 = { _v[2].x(),_v[2].y() };

	float p0 = sign(p, v0, v1);
	float p1 = sign(p, v1, v2);
	float p2 = sign(p, v2, v0);

	if (p0 > 0 && p1 > 0 && p2 > 0) return true;
	else if (p0 < 0 && p1 < 0 && p2 < 0) return true;
	return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, col_buf_id col_buffer, rst::Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	// what is f1 and f2 for ?
	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;

		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};

		//Homogeneous division
		for (auto& vec : v) {
			vec /= vec.w();
		}

		// Viewport transformation,from Canonical Cube to Screen 
		// Z is Irrelevant ,but why multiply by f1 f2
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = -vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
		//rasterize_wireframe(t);
	}
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	// iterate through the pixel and find if the current pixel is inside the triangle
	constexpr int i_max = std::numeric_limits<int>::max();
	constexpr int i_min = std::numeric_limits<int>::min();

	Vector3i bb_min = { i_max, i_max, i_max };
	Vector3i bb_max = { i_min, i_min, i_min };

	for (auto vec : v)
	{
		bb_min.x() > vec.x() ? bb_min.x() = vec.x() : bb_min.x() = bb_min.x();
		bb_min.y() > vec.y() ? bb_min.y() = vec.y() : bb_min.y() = bb_min.y();
		bb_min.z() > vec.z() ? bb_min.z() = vec.z() : bb_min.z() = bb_min.z();
		bb_max.x() < vec.x() ? bb_max.x() = vec.x() : bb_max.x() = bb_max.x();
		bb_max.y() < vec.y() ? bb_max.y() = vec.y() : bb_max.y() = bb_max.y();
		bb_max.z() < vec.z() ? bb_max.z() = vec.z() : bb_max.z() = bb_max.z();
	}

	// z is Irrelevant 
	for (int x = bb_min.x(); x < bb_max.x(); x++)
	{
		for (int y = bb_min.y(); y < bb_max.y(); y++)
		{
			if (insideTriangle(x, y, t.v)) {
				// If so, use the following code to get the interpolated z value.
				auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;
				Vector3f color = alpha * t.color[0] + beta * t.color[1] + gamma * t.color[2];
				
				if (depth_buf[get_index(x, y)] > z_interpolated)
				{
					Vector3f color = t.getColor();
					depth_buf[get_index(x, y)] = z_interpolated;
					set_pixel({ float(x),float(y),z_interpolated }, color);
				}
			}
		}
	}



	// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

