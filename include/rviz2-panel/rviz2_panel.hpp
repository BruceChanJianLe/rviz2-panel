#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
// RVIZ2
#include <rviz_common/panel.hpp>
// Qt
#include <QtWidgets>
// STL
#include <memory>
/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_push_button.h>

namespace custom_panel
{
  class RvizPushButtonPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit RvizPushButtonPanel(QWidget *parent = nullptr);
    ~RvizPushButtonPanel();

    /// Load and save configuration data
    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

  private Q_SLOTS:
    void on_pushButton1_clicked();
    void on_pushButton2_clicked();

  private:
    std::unique_ptr<Ui::gui> ui_;
    rclcpp::Node::SharedPtr node_;
    uint16_t count_button_1_, count_button_2_;

  protected:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button1_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button2_pub_;
    std_msgs::msg::Bool msg_;
  };
} // custom_panel
