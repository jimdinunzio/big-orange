import logging
import time
from awscrt import mqtt
from awsiot import mqtt_connection_builder
from threading import Thread
from orange_utils import OrangeOpType
import my_sdp_client
import sdp_comm

class AwsMqttListener(object):
    def __init__(self):
        # Configure logging
        self.logger = logging.getLogger("AWSMQTTListener")
        self.logger.setLevel(logging.WARN)

        # Parameters for AWS IoT MQTT Client.
        self.iotThingEndpoint = "a2kflp1winbg2a-ats.iot.us-west-1.amazonaws.com"
        self.iotThingPort = 8883
        self.certificatePath = "certificates/"
        self.rootCAPath = self.certificatePath + "AmazonRootCA1.pem" # eg. "AmazonRootCA1.pem"
        self.privateKeyPath = self.certificatePath + "BigOrange.private.key" # eg. "VoiceControlledRobot.private.key"
        self.certificatePath = self.certificatePath + "BigOrange.cert.pem" # eg. "VoiceControlledRobot.cert.pem"
        self.commandTopic = "command_topic"
        self.responseTopic = "response_topic"
        self.clientID = "big-orange-robot"
        self._running = False
        self._connected = False
        self._handle_op_request = None
        self._sdp : my_sdp_client.MyClient = None

        # Create a MQTT connection from the command line data
        self.mqtt_connection = mqtt_connection_builder.mtls_from_path(
            endpoint=self.iotThingEndpoint,
            port=self.iotThingPort,
            cert_filepath=self.certificatePath,
            pri_key_filepath=self.privateKeyPath,
            ca_filepath=self.rootCAPath,
            on_connection_interrupted=self.on_connection_interrupted,
            on_connection_resumed=self.on_connection_resumed,
            client_id=self.clientID,
            clean_session=False,
            keep_alive_secs=30,
            http_proxy_options=None,
            on_connection_success=self.on_connection_success,
            on_connection_failure=self.on_connection_failure,
            on_connection_closed=self.on_connection_closed)

    # thread main
    def start(self, handle_op_request, connect_sdp=True):
        # Connect
        self._handle_op_request = handle_op_request

        self.connect()
        
        if connect_sdp:
            self._sdp = my_sdp_client.MyClient()
            sdp_comm.connectToSdp(self._sdp)
        else:
            self._sdp = None

        self._running = True

        while self._running:
            time.sleep(1)        

        # Disconnect
        self.disconnect();
        print("MQTT Disconnected!")

        if self._sdp is not None:
            self._sdp.disconnect()
            self._sdp.shutdown_server32(kill_timeout=1)
            self._sdp = None

    def connect(self):
        print("MQTT: Connecting to endpoint with client ID")
        connect_future = self.mqtt_connection.connect()

        # Future.result() waits until a result is available
        result = connect_future.result()
        print("MQTT: connect result = ", result)

        retry_count = 3
        while result is Exception and retry_count > 0:
            time.sleep(1)
            print("trying to connect again")
            connect_future = self.mqtt_connection.connect()
            result = connect_future.result()
            print("MQTT: connect result = ", result)
            retry_count -= 1

        if retry_count == 0:
            return False
        self._connected = True
        
        # Subscribe
        print("Subscribing to topic '{}'...".format(self.commandTopic))
        subscribe_future, packet_id = self.mqtt_connection.subscribe(
            topic=self.commandTopic,
            qos=mqtt.QoS.AT_MOST_ONCE,
            callback=self.on_command_received)

        subscribe_result = subscribe_future.result()
        print("Subscribed with {}".format(str(subscribe_result['qos'])))
        return True

    def disconnect(self):
        if self._connected:
            disconnect_future = self.mqtt_connection.disconnect()
            disconnect_future.result()
            self._connected = False        

    # Command from MQTT message callback
    def commandCallback(self, topic, command:str):        
        # Execute command
        print("Handling MQTT command: ", command)
        if "battery" in command:
            batt_level = self._handle_op_request(self._sdp, OrangeOpType.BatteryPercent)
            response = "Big Orange's battery is at " + str(batt_level) + " percent."
        elif command == "location" or command == "where are you":
            response = "Big Orange is located " + self._handle_op_request(self._sdp, OrangeOpType.Location)
        elif command == "status":
            batt_level, loc = self._handle_op_request(self._sdp, OrangeOpType.Status)
            response = "Big Orange is located " + loc + " and his battery is at " + str(batt_level) + " percent."
        elif command.startswith("go to"):
            location = command.split("go to",1)[1].strip()
            result = self._handle_op_request(self._sdp, OrangeOpType.GotoCommand, location)
            if result:
                response = "Ok. I've asked Big Orange to go to the " + location + "."
            else:
                response = "Sorry, Big Orange does not know how to get to the " + location + "."
        elif command == "go recharge":
            self._handle_op_request(self._sdp, OrangeOpType.GotoCommand, "recharge")
            response = "Ok. I've asked Big Orange to go recharge."
        elif command == "take a picture":
            self._handle_op_request(self._sdp, OrangeOpType.TakeAPictureCommand)
            response = "Ok. Big Orange took a picture. It will be shown on his screen."
        elif command == "stop all movement":
            self._handle_op_request(self._sdp, OrangeOpType.StopAllMovement)
            response = "Ok. I told Big Orange to stop all motors."
        else:
            print("MQTT: unknown command")
            return
        self.publish_response(response)
        print("MQTT: Command processed.")

    def publish_response(self, response):            
        if not self._connected:
            self.connect()
        if self._connected:         
            self.mqtt_connection.publish(
                topic=self.responseTopic,
                payload=response,
                qos=mqtt.QoS.AT_MOST_ONCE)
                
            print("published response on topic '{}': {}".format(self.responseTopic, response))
        else:
            print("cannot publish, there is no active connection.")

    def shutdown(self):
        self._running = False

    # Callback when connection is accidentally lost.
    def on_connection_interrupted(self, connection, error, **kwargs):
        print("Connection interrupted. error: {}".format(error))
        self._connected = False
        
    # Callback when an interrupted connection is re-established.
    def on_connection_resumed(self, connection, return_code, session_present, **kwargs):
        print("Connection resumed. return_code: {} session_present: {}".format(return_code, session_present))

        if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
            print("Session did not persist. Resubscribing to existing topics...")
            resubscribe_future, _ = connection.resubscribe_existing_topics()

            # Cannot synchronously wait for resubscribe result because we're on the connection's event-loop thread,
            # evaluate result with a callback instead.
            resubscribe_future.add_done_callback(self.on_resubscribe_complete)
        self._connected = True


    def on_resubscribe_complete(self, resubscribe_future):
        resubscribe_results = resubscribe_future.result()
        print("Resubscribe results: {}".format(resubscribe_results))

        for topic, qos in resubscribe_results['topics']:
            if qos is None:
                print("Server rejected resubscribe to topic: {}".format(topic))


    # Callback when the subscribed topic receives a message
    def on_command_received(self, topic, payload, dup, qos, retain, **kwargs):
        print("Received command from topic '{}': {}".format(topic, payload))
        self.commandCallback(topic, payload.decode())

    # Callback when the connection successfully connects
    def on_connection_success(self, connection, callback_data):
        assert isinstance(callback_data, mqtt.OnConnectionSuccessData)
        print("Connection Successful with return code: {} session present: {}".format(callback_data.return_code, callback_data.session_present))
        self._connected = True

    # Callback when a connection attempt fails
    def on_connection_failure(self, connection, callback_data):
        assert isinstance(callback_data, mqtt.OnConnectionFailureData)
        print("Connection failed with error code: {}".format(callback_data.error))
        self._connected = False

    # Callback when a connection has been disconnected or shutdown successfully
    def on_connection_closed(self, connection, callback_data):
        print("Connection closed")
        self._connected = False

if __name__ == '__main__':
    def handle_op_request(sdp, opType : OrangeOpType, arg1=None, arg2=None):
        if opType == OrangeOpType.BatteryPercent:
            return 50
        elif opType == OrangeOpType.Location:
            answer = "at the office"
            return answer
        elif opType == OrangeOpType.Status:
            return 50, "at the office"
        elif opType == OrangeOpType.GotoCommand:
            return True
        elif opType == OrangeOpType.TakeAPictureCommand:
            return True

    listener = AwsMqttListener()
    t = Thread(target=listener.start, args = (handle_op_request,False,), name = "aws_mqtt_listener")
    t.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            listener.shutdown()
            t.join()
            break
