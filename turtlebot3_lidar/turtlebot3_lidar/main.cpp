// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author : Kei, Darby Lim  */

#include <string.h> //strcmp
#include <stdlib.h> //atoi
#include <stdio.h>
#include <vector>
#include <chrono>

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include "LaserScanWriter.h"
#include "lds_driver.h"

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU * STREAM_HISTORY

#define ALL_RANGES false

int main(int args, char** argv)
{
  // Lidar
  std::string port;
  int baud_rate;
  uint16_t rpms;

  port = "/dev/ttyUSB0";
  baud_rate = 230400;
  boost::asio::io_service io;

  lds::LFCDLaser laser(port, baud_rate, io);

  // Transport
  uxrUDPTransport transport;
  uxrUDPPlatform udp_platform;
  if(!uxr_init_udp_transport(&transport, &udp_platform, "127.0.0.1", 2018))
  {
    printf("Error at create transport.\n");
    return 1;
  }

  // Session
  uxrSession session;
  uxr_init_session(&session, &transport.comm, 0xCCCCDDDD);
  if(!uxr_create_session(&session))
  {
    printf("Error at create session.\n");
    return 1;
  }

  // Streams
  uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
  uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

  uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
  uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

  // Create entities
  uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
  const char* participant_xml = "<dds>"
                                  "<participant>"
                                    "<rtps>"
                                      "<name>default_xrce_participant</name>"
                                    "</rtps>"
                                  "</participant>"
                               "</dds>";
  uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0, participant_xml, UXR_REPLACE);
  uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
  const char* topic_xml = "<dds>"
                            "<topic>"
                              "<name>scan</name>"
                              "<dataType>sensor_msgs::msg::dds_::LaserScan_</dataType>"
                            "</topic>"
                          "</dds>";
  uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml, UXR_REPLACE);

  uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

  uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
  const char* datawriter_xml = "<dds>"
                                  "<data_writer>"
                                    "<topic>"
                                      "<kind>NO_KEY</kind>"
                                      "<name>rt/scan</name>"
                                      "<dataType>sensor_msgs::msg::dds_::LaserScan_</dataType>"
                                    "</topic>"
                                  "</data_writer>"
                               "</dds>";
  uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE);


  // Send create entities message and wait its status
  uint8_t status[4];
  uint16_t requests[4] = {participant_req, topic_req, publisher_req, datawriter_req};
  if(!uxr_run_session_until_all_status(&session, 1000, requests, status, 4))
  {
    printf("Error at create entities: participant: %i topic: %i publisher: %i darawriter: %i\n", status[0], status[1], status[2], status[3]);
    return 1;
  }

  // Write topics
  bool connected = true;
  uint32_t pre_time = 0;
  while(connected)
  {
    LaserScan topic;
    ucdrBuffer mb;
    std::vector<float> lidar_info = laser.poll();
    char frame_id[20];
    uint32_t topic_size;
	  sprintf(frame_id, "base_scan");
	  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();

  	topic.header.stamp.sec = now.count() / 1000000000;
  	topic.header.stamp.nanosec = now.count() % 1000000000;

  	strcpy(topic.header.frame_id, frame_id);

    topic.angle_increment = (2.0*M_PI/360.0);
	  topic.time_increment = lidar_info.at(lidar_info.size()-1);
    topic.angle_min = 0.0f;
    topic.angle_max = 2.0*M_PI-topic.angle_increment;
	  topic.scan_time = 0.0f;
    topic.range_min = 0.12f;
    topic.range_max = 3.5;

	  if (ALL_RANGES)
    {
	    topic.ranges_size = 360;
    }
	  else
    {
      topic.ranges_size = 180;
    }
	  
    topic.intensities_size = 1;

	  if (ALL_RANGES)
	  {
      for (uint16_t index = 0; index < (lidar_info.size()-1); index++)
  	  {
        topic.ranges[(topic.ranges_size-1)-index] = lidar_info.at(index);
	    }
	  }
	  else
	  {
      for (uint16_t index = 0; index < ((lidar_info.size()-1)/2); index++)
      {
        topic.ranges[(topic.ranges_size-1)-index] = lidar_info.at(index*2);
      }
	  }
    
    topic_size = LaserScan_size_of_topic(&topic, 0);
    uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &mb, topic_size);
    LaserScan_serialize_topic(&mb, &topic);

    connected = uxr_run_session_until_timeout(&session, 200);
    //connected = mr_run_session_until_confirm_delivery(&session, 200);
	  if(connected)
    {
      //printf("Sent topic: %f, range_max: %d\n", lidar_info.at(1), lidar_info.size());
    }
  } 

  // Delete resources
  laser.close();
  uxr_delete_session(&session);
  uxr_close_udp_transport(&transport);

  return 0;
}
