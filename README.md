## Using this repo

To run the simulator, open two terminals. Then you will need to run the following scripts one in each terminal. In a conda prompt shell, the following commands 
would work.

```
python broker.py
python simulator.py which_map.json
```


## broker.py and simulator.py

The broker script makes the ZeroMQ messaging easier. All nodes publish to the
broker and subscribe to the broker. The broker forwards all messages to all the
nodes. This way you don't have to manage polling multiple sockets to
communicate with multiple nodes.

The simulator script does the following:

* listen for inputs on the `wheel_speeds` topic
* add noise to the wheel speeds
* run the forward kinematics
* check for collisions and output a message on the `collision` topic if a collision occurs
* simulate the lidar and output a message on the `lidar` topic with sensor measurements
* simulate landmark detection and output a message on the `landmarks` topic with detected landmarks
* draw a nice illustration of the robot in the environment

## Sending and receiving ZeroMQ messages

A sample controller can be found in the `test_controller.py` script. This shows
how to read sensor data and publish wheel speeds. The following section
provides a little more detail.

To communicate with the broker, first create a ZeroMQ context
```python
import zmq

context = zmq.Context()
```

Then, to publish messages, create a socket and send the message. The message
must first be serialized. One way to do this is to create a dictionary with the
different items in it and then serialize that dictionary to a json string using
the `dumps()` function. Once that has been accomplished, the string can be
converted into raw bytes with the `encode()` function.

```python
pub_socket = context.socket(zmq.PUB)

# pub_socket.connect("ipc://file_path/pub.ipc") #-----  Alternative approach using files instead of TCP.
pub_socket.connect("tcp://localhost:5557") 


message_dict = {"key1": 20, "key2": "a string"}
message_str = json.dumps(message_dict)
message_encoded = message_str.encode()
pub_socket.send_multipart([b"my_nifty_topic", message_encoded])
```

To receive messages, you have to create a socket and specify which topics
you're interested in. After that, you can receive the message, decode it, and
convert it back to a dictionary using the `loads()` function.

```python
sub_socket = context.socket(zmq.SUB)
# sub_socket.connect("ipc:///tmp/robo_sim/sub.ipc") #-----  Alternative approach using files instead of TCP.
sub_socket.connect("tcp://localhost:5555") 

sub_socket.setsockopt(zmq.SUBSCRIBE, b"my_nifty_topic")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"second_nifty_topic")

topic, message_str = sub_socket.recv_multipart()
message_dict = json.loads(message_str.decode())
```

If you have gotten behind on messages for some reason, you may want to read all
the messages until the queue is empty. This could be done as follows:

```python
queue_empty = False
while not queue_empty:
    try:
        topic, message_str = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
    except zmq.ZMQError:
        queue_empty = True
message_dict = json.loads(message_str.decode())
#do something with message_dict
```

## Map Generator

You can use the `map_generator.py` script to generate maps. Just run the
`map_generator.py output_file.json` script and follow the instructions that are
printed.
