// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


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

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // barycentric coordinates check inside
    Vector2f v0(_v[2].x()-_v[0].x(), _v[2].y()-_v[0].y());
    Vector2f v1(_v[1].x()-_v[0].x(), _v[1].y()-_v[0].y());
    Vector2f v2(x-_v[0].x(), y-_v[0].y());

    float dotvalue00 = v0.dot(v0);
    float dotvalue01 = v0.dot(v1);
    float dotvalue02 = v0.dot(v2);
    float dotvalue11 = v1.dot(v1);
    float dotvalue12 = v1.dot(v2);

    float inverse = 1 / (dotvalue00*dotvalue11 - dotvalue01*dotvalue01);
    float u_value = (dotvalue11*dotvalue02 - dotvalue01 * dotvalue12) * inverse;
    float v_value = (dotvalue00*dotvalue12 - dotvalue01 * dotvalue02) * inverse;

    if (u_value>=0 && u_value<=1 && v_value>=0 && v_value<=1 && u_value + v_value<=1)
        return true;
    
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

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
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
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
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    float min_x = std::min(std::min(t.v[0].x(), t.v[1].x()), t.v[2].x());
    float min_y = std::min(std::min(t.v[0].y(), t.v[1].y()), t.v[2].y());
    float max_x = std::max(std::max(t.v[0].x(), t.v[1].x()), t.v[2].x());
    float max_y = std::max(std::max(t.v[0].y(), t.v[1].y()), t.v[2].y());

    int d_min_x = std::max(0, int(min_x-1));
    int d_max_x = std::min(width-1, int(max_x+1));
    int d_min_y = std::max(0, int(min_y-1));
    int d_max_y = std::min(height-1, int(max_y+1));
    
     d_min_x = std::max(0, int(min_x-1));
     d_max_x = std::min(width-1, int(max_x+1));
     d_min_y = std::max(0, int(min_y-1));
     d_max_y = std::min(height-1, int(max_y+1));


    // do loop of bounding box
    for(int itx=d_min_x; itx<=d_max_x; itx++)
    {
        for (int ity=d_min_y; ity<=d_max_y; ity++)
        {
            bool super_samping = true;
            if (!super_samping)
            {

                if (!insideTriangle(itx, ity, t.v)) continue;
                // get deepth value
                auto[alpha, beta, gamma] = computeBarycentric2D(itx, ity, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                if (z_interpolated < depth_buf[get_index(itx, ity)])
                {
                    depth_buf[get_index(itx, ity)] = z_interpolated;
                    Eigen::Vector3f point = Eigen::Vector3f(itx, ity, 1.0f);
                    set_pixel(point, t.getColor());
                }
            }
            else
            {
                //if (!insideTriangle(itx, ity, t.v)) continue;

                //skip draw
                Eigen::Vector3f target_color(0,0,0);
                float target_deepth = 0;
                int inside_counter = 0;
                for (int sample_index = 0; sample_index<4; sample_index++)
                {
                    float sp_x, sp_y;
                    switch (sample_index)
                    {
                        case 0:
                            sp_x = float(itx)+0.25f;
                            sp_y = float(ity)+0.25f;
                            break;
                        case 1:
                            sp_x = float(itx)+0.25f;
                            sp_y = float(ity)+0.75f;
                            break;
                         case 2:
                            sp_x = float(itx)+0.75f;
                            sp_y = float(ity)+0.25f;
                            break;                           
                        case 3:
                            sp_x = float(itx)+0.75f;
                            sp_y = float(ity)+0.75f;
                            break;
                    }
                        //printf("%d", sp_x);
                        // 
                    if (!insideTriangle(sp_x, sp_y, t.v)) 
                    {
                        //target_color += (frame_buf[get_index(int(itx), int(ity))] * 0.25f);
                        continue;
                    }
                    // get deepth value
                    auto[alpha, beta, gamma] = computeBarycentric2D(sp_x, sp_y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    //z_interpolated *= w_reciprocal;
                    // try get total deep of pixel
                    target_deepth += z_interpolated;

                    inside_counter++;
                    // check each sample deepth
                    if (z_interpolated < depth_buf[get_index(int(itx), int(ity)) +( width * height * sample_index)])
                    {
                        //inside_counter++;
                        depth_buf[get_index(int(itx), int(ity)) +( width * height * sample_index)] = z_interpolated;
                        target_color += (t.getColor() * 0.25f);
                    }
                    else
                    {
                        //target_color += (frame_buf[get_index(int(itx), int(ity))] * 0.25f);
                    }  
                }
                target_deepth = target_deepth / inside_counter;
                Eigen::Vector3f point = Eigen::Vector3f(itx, ity, 1.0f);

                //if (inside_counter > 0)
                target_color += frame_buf[get_index(int(itx), int(ity))];
                set_pixel(point, target_color);


            }


        }
    }

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
    depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on