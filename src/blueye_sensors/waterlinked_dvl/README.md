## New Parser Features

The updated parser introduces the following changes:

- **NTP Server Integration**: The parser now expects the use of an NTP server for synchronizing the DVL timestamps. If time_of_validity from DVL is 0, it falls back to ROS2 time.

- **Line-by-Line JSON Parsing**: Instead of processing the entire JSON payload using `socket.recv(2048)`, the parser now reads the JSON data line by line.

- **multi-threading**: a separate thread runs the socket processing node instead of a timer.

Note: There is currently a new DVL driver that publishes ROS2 topics on Water Linked website, we didn't test it tho, so maybe check it out.