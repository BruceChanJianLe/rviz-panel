#include <gtest/gtest.h>
#include "rviz-panel/rviz_panel.hpp"


namespace rviz_panel
{
    class testClass : public ::testing::Test, public simplePanel
    {
        public:
            testClass()
            {
                ;
            }
            ~testClass()
            {
                ;
            }

            FRIEND_TEST(testClass, buttonTest);
    };

} // namespace rviz_panel

TEST_F(rviz_panel::testClass, buttonTest)
{
    ;
}

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