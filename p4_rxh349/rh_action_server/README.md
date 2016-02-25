# rh_action_server

This package contains three folders: action, launch and src.

action contains custom message for action client and server.

src contains three files:

rh_action_client.cpp should run as: rosrun rh_action_server rh_action_client

rh_action_server_w_fdbk.cpp should run as: rosrun rh_action_server rh_action_server_w_fdbk

new_lidar_alarm.cpp should run as: rosrun rh_action_server rh_action_server new_lidar_alarm

launch is trying to start three files the same time, just for convenience.

## Example usage

## Running tests/demos
    