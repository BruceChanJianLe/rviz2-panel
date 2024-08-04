#include "rviz2-panel/rviz2_panel.hpp"

namespace custom_panel
{
  RvizPushButtonPanel::RvizPushButtonPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui>())
    , node_{nullptr}
    , count_button_1_{0}
    , count_button_2_{0}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_push_button_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    button1_pub_ = node_->create_publisher<std_msgs::msg::Bool>("button1", 1);
    button2_pub_ = node_->create_publisher<std_msgs::msg::Bool>("button2", 1);

    // Prepare msg
    msg_.data = true;
  }

  RvizPushButtonPanel::~RvizPushButtonPanel()
  {
  }
    void RvizPushButtonPanel::load(const rviz_common::Config &config)
    {
      Panel::load(config);
      if (auto push_button_config = config.mapGetChild({"PushButton"}); push_button_config.isValid())
      {
        if (QVariant count_button_1 {0}; push_button_config.mapGetValue({"count_button_1"}, &count_button_1))
        {
          RCLCPP_INFO_STREAM(node_->get_logger(), "Button 1 was pressed " << count_button_1.toDouble() << " the last time.");
        }
        if (QVariant count_button_2 {0}; push_button_config.mapGetValue({"count_button_2"}, &count_button_2))
        {
          RCLCPP_INFO_STREAM(node_->get_logger(), "Button 2 was pressed " << count_button_2.toDouble() << " the last time.");
        }
      }
    }

    void RvizPushButtonPanel::save(rviz_common::Config config) const
    {
      Panel::save(config);
      rviz_common::Config push_button_config = config.mapMakeChild({"PushButton"});
      push_button_config.mapSetValue({"count_button_1"}, {count_button_1_});
      push_button_config.mapSetValue({"count_button_2"}, {count_button_2_});
    }

    void RvizPushButtonPanel::on_pushButton1_clicked()
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Published to button1 topic!");
      button1_pub_->publish(msg_);
      ui_->label_1->setText(QString::fromStdString({std::string{"Button 1 was clicked "} + std::to_string(++count_button_1_)}));
    }

    void RvizPushButtonPanel::on_pushButton2_clicked()
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Published to button2 topic!");
      button2_pub_->publish(msg_);
      ui_->label_2->setText(QString::fromStdString({std::string{"Button 2 was clicked "} + std::to_string(++count_button_2_)}));
    }

} // custom_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_panel::RvizPushButtonPanel, rviz_common::Panel)
