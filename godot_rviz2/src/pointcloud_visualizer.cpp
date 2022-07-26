/*************************************************************************/
/*  immediate_geometry.cpp                                               */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2022 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2022 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#include "pointcloud_visualizer.h"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <string>

void PointCloudVisualizer::begin(Mesh::PrimitiveType p_primitive, const Ref<Texture> &p_texture) {
	VS::get_singleton()->immediate_begin(im, (VS::PrimitiveType)p_primitive, p_texture.is_valid() ? p_texture->get_rid() : RID());
	if (p_texture.is_valid()) {
		cached_textures.push_back(p_texture);
	}
}

void PointCloudVisualizer::set_normal(const Vector3 &p_normal) {
	VS::get_singleton()->immediate_normal(im, p_normal);
}

void PointCloudVisualizer::set_tangent(const Plane &p_tangent) {
	VS::get_singleton()->immediate_tangent(im, p_tangent);
}

void PointCloudVisualizer::set_color(const Color &p_color) {
	VS::get_singleton()->immediate_color(im, p_color);
}

void PointCloudVisualizer::set_uv(const Vector2 &p_uv) {
	VS::get_singleton()->immediate_uv(im, p_uv);
}

void PointCloudVisualizer::set_uv2(const Vector2 &p_uv2) {
	VS::get_singleton()->immediate_uv2(im, p_uv2);
}

void PointCloudVisualizer::add_vertex(const Vector3 &p_vertex) {
	VS::get_singleton()->immediate_vertex(im, p_vertex);
	if (empty) {
		aabb.position = p_vertex;
		aabb.size = Vector3();
		empty = false;
	} else {
		aabb.expand_to(p_vertex);
	}
}

void PointCloudVisualizer::visualize_latest_pointcloud()
{
	if (!is_new_ || msg_ptr_ == nullptr)
		return;

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg_ptr_, "x"),
		iter_y(*msg_ptr_, "y"), iter_z(*msg_ptr_, "z");
	clear();

	// Divide pointcloud because it cannot visualize large pointcloud, I don't know why
#if 1
	while (iter_x != iter_x.end())
	{
		begin(Mesh::PRIMITIVE_POINTS);
		int count = 0;
		for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
		{
			if (500 <= count)
				break;
			add_vertex(Vector3(*iter_x, *iter_z, -1.0 * (*iter_y)));
			count++;
		}
		end();
	}
#else
	// begin(Mesh::PRIMITIVE_POINTS);
	// for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
	// {
	// 	add_vertex(Vector3(*iter_x, *iter_z, -1.0 * (*iter_y)));
	// }
	// end();
#endif

	is_new_ = false;
}

void PointCloudVisualizer::subscribe(const String &topic, const bool transient_local)
{
	std::wstring ws = topic.c_str();
	std::string s(ws.begin(), ws.end());
	if (transient_local)
		subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<sensor_msgs::msg::PointCloud2>(
			s, rclcpp::QoS{1}.transient_local(),
			std::bind(&PointCloudVisualizer::on_pointcloud2, this, std::placeholders::_1));
	else
		subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<sensor_msgs::msg::PointCloud2>(
			s, rclcpp::SensorDataQoS().keep_last(1),
			std::bind(&PointCloudVisualizer::on_pointcloud2, this, std::placeholders::_1));
}

void PointCloudVisualizer::end() {
	VS::get_singleton()->immediate_end(im);
}

void PointCloudVisualizer::clear() {
	VS::get_singleton()->immediate_clear(im);
	empty = true;
	cached_textures.clear();
}

AABB PointCloudVisualizer::get_aabb() const {
	return aabb;
}
PoolVector<Face3> PointCloudVisualizer::get_faces(uint32_t p_usage_flags) const {
	return PoolVector<Face3>();
}

// void PointCloudVisualizer::add_sphere(int p_lats, int p_lons, float p_radius, bool p_add_uv) {
// 	for (int i = 1; i <= p_lats; i++) {
// 		double lat0 = Math_PI * (-0.5 + (double)(i - 1) / p_lats);
// 		double z0 = Math::sin(lat0);
// 		double zr0 = Math::cos(lat0);

// 		double lat1 = Math_PI * (-0.5 + (double)i / p_lats);
// 		double z1 = Math::sin(lat1);
// 		double zr1 = Math::cos(lat1);

// 		for (int j = p_lons; j >= 1; j--) {
// 			double lng0 = 2 * Math_PI * (double)(j - 1) / p_lons;
// 			double x0 = Math::cos(lng0);
// 			double y0 = Math::sin(lng0);

// 			double lng1 = 2 * Math_PI * (double)(j) / p_lons;
// 			double x1 = Math::cos(lng1);
// 			double y1 = Math::sin(lng1);

// 			Vector3 v[4] = {
// 				Vector3(x1 * zr0, z0, y1 * zr0),
// 				Vector3(x1 * zr1, z1, y1 * zr1),
// 				Vector3(x0 * zr1, z1, y0 * zr1),
// 				Vector3(x0 * zr0, z0, y0 * zr0)
// 			};

// #define ADD_POINT(m_idx)                                                                                    \
// 	if (p_add_uv) {                                                                                         \
// 		set_uv(Vector2(Math::atan2(v[m_idx].x, v[m_idx].z) / Math_PI * 0.5 + 0.5, v[m_idx].y * 0.5 + 0.5)); \
// 		set_tangent(Plane(Vector3(-v[m_idx].z, v[m_idx].y, v[m_idx].x), 1));                                \
// 	}                                                                                                       \
// 	set_normal(v[m_idx]);                                                                                   \
// 	add_vertex(v[m_idx] * p_radius);

// 			ADD_POINT(0);
// 			ADD_POINT(1);
// 			ADD_POINT(2);

// 			ADD_POINT(2);
// 			ADD_POINT(3);
// 			ADD_POINT(0);
// 		}
// 	}
// }

void PointCloudVisualizer::_bind_methods() {
	// ClassDB::bind_method(D_METHOD("begin", "primitive", "texture"), &PointCloudVisualizer::begin, DEFVAL(Ref<Texture>()));
	ClassDB::bind_method(D_METHOD("set_normal", "normal"), &PointCloudVisualizer::set_normal);
	ClassDB::bind_method(D_METHOD("set_tangent", "tangent"), &PointCloudVisualizer::set_tangent);
	ClassDB::bind_method(D_METHOD("set_color", "color"), &PointCloudVisualizer::set_color);
	ClassDB::bind_method(D_METHOD("set_uv", "uv"), &PointCloudVisualizer::set_uv);
	ClassDB::bind_method(D_METHOD("set_uv2", "uv"), &PointCloudVisualizer::set_uv2);
	// ClassDB::bind_method(D_METHOD("add_vertex", "position"), &PointCloudVisualizer::add_vertex);
	ClassDB::bind_method(D_METHOD("visualize_latest_pointcloud"), &PointCloudVisualizer::visualize_latest_pointcloud);
	ClassDB::bind_method(D_METHOD("subscribe", "topic", "transient_local"), &PointCloudVisualizer::subscribe);
	// ClassDB::bind_method(D_METHOD("add_sphere", "lats", "lons", "radius", "add_uv"), &PointCloudVisualizer::add_sphere, DEFVAL(true));
	// ClassDB::bind_method(D_METHOD("end"), &PointCloudVisualizer::end);
	// ClassDB::bind_method(D_METHOD("clear"), &PointCloudVisualizer::clear);
}

void PointCloudVisualizer::on_pointcloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
	msg_ptr_ = msg;
	is_new_ = true;
}

PointCloudVisualizer::PointCloudVisualizer()
{
	im = VisualServer::get_singleton()->immediate_create();
	set_base(im);
	empty = true;
	is_new_ = false;
}

PointCloudVisualizer::~PointCloudVisualizer() {
	VisualServer::get_singleton()->free(im);
}
