#include <gtest/gtest.h>
#include "rviz-panel/rviz_panel.hpp"
#include <ros/callback_queue.h>
#include <thread>


namespace rviz_panel
{
    class testClass : public ::testing::Test, public simplePanel
    {
        friend class msgDataTestClass;
        public:
            bool msg_return_value_;
        public:
            testClass()
            {
                ;
            }
            ~testClass()
            {
                ;
            }

            // Sets up the test fixture.
            virtual void SetUp() override
            {
                ;
            }

            // Tears down the test fixture.
            virtual void TearDown() override
            {
                ;
            }
    };

    /**
     * @brief Setup parameterize data structure
     * 
     */
    struct message_data
    {
        bool data;
    };


    class msgDataTestClass : public testClass, public ::testing::WithParamInterface<rviz_panel::message_data>
    {
        public:
            msgDataTestClass()
            {
                ;
            }
            ~msgDataTestClass()
            {
                ;
            }

            ros::Subscriber button_1_sub_;
            ros::Subscriber button_2_sub_;

            // Sets up the test fixture.
            virtual void SetUp() override
            {
                // Setup ROS node handle
                nh_ = ros::NodeHandle();

                // Setup publishers and subscribers
                button_1_pub_ = nh_.advertise<std_msgs::Bool>("button_1_topic", 1, true);
                button_2_pub_ = nh_.advertise<std_msgs::Bool>("button_2_topic", 1, true);
                
                auto callback = [this](const std_msgs::Bool::ConstPtr & msg)
                {
                    // Set receive value to return value
                    this->msg_return_value_ = msg->data;
                };

                button_1_sub_ = nh_.subscribe<std_msgs::Bool>("button_1_topic", 1, callback);
                button_2_sub_ = nh_.subscribe<std_msgs::Bool>("button_2_topic", 1, callback);
            }

            // Tears down the test fixture.
            virtual void TearDown() override
            {
                ;
            }

            FRIEND_TEST(msgDataTestClass, buttonTest);
    };


    TEST_P(msgDataTestClass, buttonTest)
    {
        // Set message value from getParam()
        this->msg_.data = GetParam().data;

        // Call QT slot function like this (but in this example it does not work)
        // Hence publish myself
        SLOT(this->button_one());
        button_1_pub_.publish(this->msg_); // publishing myself instead

        ros::Rate r(1);
        while (nh_.ok())
        {
            if(this->msg_return_value_ != this->msg_.data)
            {
                ros::spinOnce();
                r.sleep();
            }
            else
            {
                break;
            }
        }

        EXPECT_EQ(GetParam().data, this->msg_return_value_);
    }


    INSTANTIATE_TEST_CASE_P(
        Default,
        msgDataTestClass,
        ::testing::Values(
            message_data{true},
            message_data{false}
        )
    );

} // namespace rviz_panel


int main(int argc, char ** argv)
{
    // Initialize gtest
    testing::InitGoogleTest(&argc, argv);

    // Initialize Qt Application
    QApplication app(argc, argv);

    // Initialize ROS node
    ros::init(argc, argv, "rviz_panel_test_node");

    // Run all test
    return RUN_ALL_TESTS();
}