#!/bin/bash
rosbag filter input.bag output.bag "t.to_sec() < 2884.772651110 or t.to_sec() > 2884.775651110"
