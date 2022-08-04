//
//  Copyright 2022 Yukihiro Saito. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#pragma once

#include "core/ustring.h"
#include "core/variant.h"
#include "core/reference.h"
#include "godot_rviz2.hpp"
#include "util.hpp"
#include <optional>
#include "rclcpp/qos.hpp"

template <class T>
class TopicSubscriber : public Reference
// class TopicSubscriber
{
	GDCLASS(TopicSubscriber, Reference);
	// GDCLASS(TopicSubscriber<T>, Reference);

private:
	using ConstSharedPtr = typename T::ConstSharedPtr;

	typename rclcpp::Subscription<T>::SharedPtr subscription_;
	void on_callback(const ConstSharedPtr msg)
	{
		msg_ptr_ = msg;
		is_new_ = true;
	}

	ConstSharedPtr msg_ptr_;
	bool is_new_;

public:
	bool is_new() { return is_new_; }
	void set_old()
	{
		is_new_ = false;
	}
	std::optional<ConstSharedPtr> get_last_msg()
	{
		if (!msg_ptr_)
			return std::nullopt;
		return msg_ptr_;
	}

	void subscribe(const String &topic, const bool transient_local = false)
	{
		rclcpp::QoS &qos = rclcpp::SensorDataQoS().keep_last(1);
		if (transient_local)
			qos = rclcpp::QoS{1}.transient_local();

		subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<T>(
			godot_to_std(topic), qos, std::bind(&TopicSubscriber::on_callback, this, std::placeholders::_1));
	}

	TopicSubscriber()
	{
		is_new_ = false;
	}
	virtual ~TopicSubscriber() = default;
protected:
	static void _bind_methods()
	{
		ClassDB::bind_method(D_METHOD("subscribe"), &TopicSubscriber::subscribe);
		ClassDB::bind_method(D_METHOD("is_new"), &TopicSubscriber::is_new);
		ClassDB::bind_method(D_METHOD("set_old"), &TopicSubscriber::set_old);
		// ClassDB::bind_method(D_METHOD("subscribe"), &TopicSubscriber<T>::subscribe);
		// ClassDB::bind_method(D_METHOD("is_new"), &TopicSubscriber<T>::is_new);
		// ClassDB::bind_method(D_METHOD("set_old"), &TopicSubscriber<T>::set_old);
	}
};
