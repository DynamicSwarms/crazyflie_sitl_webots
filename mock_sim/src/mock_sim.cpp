#include <rclcpp/rclcpp.hpp>
#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "crtp_interfaces/msg/crtp_packet.hpp"

#define CRTP_PORT_LOCALIZATION 0x06
#define CRTP_PORT_SETPOINT_SIM 0x09

#define LOCALIZATION_POSITION_CHANNEL 0
#define LOCALIZATION_GENERIC_CHANNEL 1

#define LOCALIZATION_GENERIC_EXT_POSE_TYPE 8

// Sensor type (first byte of crtp packet)
enum SensorTypeSim_e {
  SENSOR_GYRO_ACC_SIM   = 0,
  SENSOR_MAG_SIM        = 1,
  SENSOR_BARO_SIM       = 2,
};


union Axis3i16 {
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
}__attribute__((packed));

union Axis3f {
	struct {
		float x;
		float y;
		float z;
	};
	float axis[3];
}__attribute__((packed));

struct baro_s {
	uint8_t type;
	float pressure;           // mbar
	float temperature;        // degree Celcius
	float asl;                // m (ASL = altitude above sea level)
} __attribute__((packed));

struct imu_s {
	uint8_t type;
	union Axis3i16 acc;
	union Axis3i16 gyro;
} __attribute__((packed));

struct mag_s {
	uint8_t type;
	union Axis3i16 mag;
} __attribute__((packed));

struct pos_s {
	float x; 
  float y;
  float z;

} __attribute__((packed));

struct pose_s
{
  float x; // in m
  float y; // in m
  float z; // in m
  float qx;
  float qy;
  float qz;
  float qw;
} __attribute__((packed));

class MockSimNode : public rclcpp::Node
{
public:
  MockSimNode()
  : Node("mock_sim_node")
  {
    // Timer at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&MockSimNode::timerCallback, this)
    );

    // Client
    client_ = this->create_client<crtp_interfaces::srv::CrtpPacketSend>("crazyradio/send_crtp_packet80");

    RCLCPP_INFO(this->get_logger(), "MockSimNode has been started.");
  }

private:

  crtp_interfaces::msg::CrtpPacket get_barometer()
  {
    crtp_interfaces::msg::CrtpPacket packet;
    packet.port = CRTP_PORT_SETPOINT_SIM;
    packet.channel = 0;
    packet.data_length = sizeof(baro_s);
    baro_s baro_data;
    baro_data.type = SENSOR_BARO_SIM;
    baro_data.pressure = 1013.25; // mbar
    baro_data.temperature = 25.0; // degree Celcius
    baro_data.asl = 100.0; // m (ASL = altitude above sea level)
    memcpy(packet.data.data(), &baro_data, sizeof(baro_s));
    return packet;
  }

  crtp_interfaces::msg::CrtpPacket get_imu()
  {
    crtp_interfaces::msg::CrtpPacket packet;
    packet.port = CRTP_PORT_SETPOINT_SIM;
    packet.channel = 0;
    packet.data_length = sizeof(imu_s);
    imu_s imu_data;
    imu_data.type = SENSOR_GYRO_ACC_SIM;
    imu_data.acc.x = 0.0; // m/s^2
    imu_data.acc.y = 0.0; // m/s^2
    imu_data.acc.z = 2048.0; //-9.81; // m/s^2
    imu_data.gyro.x = 0.0; // deg/s
    imu_data.gyro.y = 0.0; // deg/s
    imu_data.gyro.z = 0.0; // deg/s
    memcpy(packet.data.data(), &imu_data, sizeof(imu_s));
    return packet;
  }

  crtp_interfaces::msg::CrtpPacket get_mag()
  {
    crtp_interfaces::msg::CrtpPacket packet;
    packet.port = CRTP_PORT_SETPOINT_SIM;
    packet.channel = 0;
    packet.data_length = sizeof(mag_s);
    mag_s mag_data;
    mag_data.type = SENSOR_MAG_SIM;
    mag_data.mag.x = 0.0; // uT
    mag_data.mag.y = 0.0; // uT
    mag_data.mag.z = 0.0; // uT
    memcpy(packet.data.data(), &mag_data, sizeof(mag_s));
    return packet;
  }

  crtp_interfaces::msg::CrtpPacket get_pos()
  {
    crtp_interfaces::msg::CrtpPacket packet;
    packet.port = CRTP_PORT_LOCALIZATION;
    packet.channel = LOCALIZATION_GENERIC_CHANNEL;
    packet.data_length = sizeof(pose_s) + 1; // +1 for the type of the generic packet
    pose_s pose_data;
    pose_data.x = 0.0; // m
    pose_data.y = 0.0; // m
    pose_data.z = 0.0; // m
    pose_data.qx = 0.0;
    pose_data.qy = 0.0;
    pose_data.qz = 0.0;
    pose_data.qw = 1.0;
    packet.data[0] = LOCALIZATION_GENERIC_EXT_POSE_TYPE; // first byte is the type of the generic packet
    memcpy(packet.data.data() + 1, &pose_data, sizeof(pose_s));
    return packet;
    //packet.port = CRTP_PORT_LOCALIZATION;
    //packet.channel = LOCALIZATION_POSITION_CHANNEL;
    //packet.data_length = sizeof(pos_s);
    //pos_s pos_data;
    //pos_data.x = 0.0; // m
    //pos_data.y = 0.0; // m
    //pos_data.z = 0.0; // m
    //memcpy(packet.data.data(), &pos_data, sizeof(pos_s));
    //return packet;
  }
  



  void timerCallback()
  {
    static int n_th_call = 0;
    n_th_call++;
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service 'crazyradio/send_crtp_packet80' not available.");
      return;
    }

    auto request = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    request->link.channel = 80;
    memcpy(request->link.address.data(), "\xE7\xE7\xE7\xE7\xE7", 5); // broadcast address
    
    request->packet = get_barometer();
    auto future = client_->async_send_request(request);

    request->packet = get_imu();
    if (n_th_call == 100) // adter 1 second, send a burst of 1000 mag packets to test the queueing and dropping of packets in the crazyradio node
    {
      for (int i= 0; i < 1000; i++)
        client_->async_send_request(request);
    }
    future = client_->async_send_request(request);

    request->packet = get_mag();
    future = client_->async_send_request(request);

    request->packet = get_pos();
    future = client_->async_send_request(request);

    // Use a parameter client to set the parameter in another node
    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "cf231");
    if (n_th_call % 100 ==0 && parameter_client->service_is_ready()) {
      parameter_client->set_parameters({
        //rclcpp::Parameter("locSrv.extPosStdDev", 1e-3)
      });
    } else {
      //RCLCPP_WARN(this->get_logger(), "Parameter client for 'cf231' not available.");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
