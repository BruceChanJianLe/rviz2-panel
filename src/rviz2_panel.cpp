#include "rviz2-panel/rviz2_panel.hpp"
#include <rviz_common/display_context.hpp>

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
  }

  RvizPushButtonPanel::~RvizPushButtonPanel()
  {
  }

  void RvizPushButtonPanel::onInitialize()
  {
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
    // (as per normal rclcpp code)
    node_ = node_ptr_->get_raw_node();

    button1_pub_ = node_->create_publisher<std_msgs::msg::Bool>("button1", 1);
    button2_pub_ = node_->create_publisher<std_msgs::msg::Bool>("button2", 1);

    // Prepare msg
    msg_.data = true;
  }

  void RvizPushButtonPanel::load(const rviz_common::Config &config)
  {
    Panel::load(config);
    if (auto push_button_config = config.mapGetChild({"PushButton"}); push_button_config.isValid())
    {
      if (int count_button_1{0}; push_button_config.mapGetInt({"count_button_1"}, &count_button_1))
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Button 1 was pressed " << count_button_1 << " the last time.");
      }
      if (int count_button_2{0}; push_button_config.mapGetInt({"count_button_2"}, &count_button_2))
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Button 2 was pressed " << count_button_2 << " the last time.");
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
    ui_->label_1->setText(QString::number(++count_button_1_));
  }

  void RvizPushButtonPanel::on_pushButton2_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Published to button2 topic!");
    button2_pub_->publish(msg_);
    ui_->label_2->setText(QString::number(++count_button_2_));
  }

} // custom_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_panel::RvizPushButtonPanel, rviz_common::Panel)
