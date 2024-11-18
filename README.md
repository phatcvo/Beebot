## Nodes

| Node                | Method                             |
| ------------------- | ---------------------------------- |
| local_map_creator   | Ray casting update                 |
| localization        | emcl: mcl with expansion resetting |
| global_path_planner | A\* search algorithm               |
| local_path_planner  | DWA: Dynamic Window Approach       |

## How to run

```
export TURTLEBOT3_MODEL=burger
roslaunch system_ros demo.launch

rosrun rosserial_python serial_node.py /dev/ttyACM0
```

# mqtt transmit

### Step 1: Install Mosquitto as the MQTT broker (Raspberry Pi)

```
`sudo apt install mosquitto mosquitto-clients`

`sudo systemctl enable mosquitto`

`sudo systemctl start mosquitto`

```

- Check the status of Mosquitto:

  `sudo systemctl status mosquitto`
  

  If it’s running, see something like:

  `Active: active (running) ...`

### Step 2: Configure Mosquitto

`sudo nano /etc/mosquitto/mosquitto.conf`

Enable password authentication by adding this line to the config file:

`listener <port_number>`
`allow_anonymous false`
`password_file /etc/mosquitto/pwfile`

Set password by:
`sudo mosquitto_passwd -c /etc/mosquitto/pwfile <username>`

Replace `<username>` with the name you want to use. You will be prompted to set a password.

Verify that the password file exists:
`cat /etc/mosquitto/pwfile`
Should see something like this (hashed password):
`<username>:$6$xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx`

(Optional)
Enable TLS (for secure connections): If you want to enable encryption (TLS) for MQTT communication:

`cafile /etc/mosquitto/certs/ca.crt`

`certfile /etc/mosquitto/certs/server.crt`

`keyfile /etc/mosquitto/certs/server.key`

You would need to generate or obtain SSL/TLS certificates for this, which can be self-signed or from a certificate authority.

Restart Mosquitto for the changes to take effect:
`sudo systemctl restart mosquitto`

### Step 3: Install MQTT Client Tools on PCs (Clients)

1. Install Mosquitto Clients

`sudo apt install mosquitto-clients`
`sudo apt install libmosquitto-dev`

### Step 4: Test the Connection between PCs (Clients) and Raspberry Pi (Broker)

Test MQTT Publish from the PC1 (Client):

`mosquitto_pub -h <broker_ip> -t "test/topic" -m "Hello from PC!" -u <username> -P <password>`

Replace <broker_ip> with the actual IP address of your Raspberry Pi.

Test MQTT Subscribe from the PC2 (Client):

Open another terminal and subscribe to the same topic:

`mosquitto_sub -h <broker_ip> -t "test/topic" -u <username> -P <password>`

Should see the message "Hello from PC!" appear in the terminal of the subscribing client.


Ref: https://mosquitto.org/api/files/mosquitto-h.html
