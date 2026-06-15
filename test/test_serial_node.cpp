#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosserial_server/serial_session.hpp>

class TestSerialSession : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(TestSerialSession, InitializeSession) {
  auto session = std::make_shared<rosserial_server::SerialSession>(
    boost::asio::io_context(), "test_port", 115200);
  
  EXPECT_NO_THROW({
    session->start();
  });
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}