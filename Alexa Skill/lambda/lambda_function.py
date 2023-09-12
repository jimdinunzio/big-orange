# -*- coding: utf-8 -*-

# This code includes handling intents from an Alexa skill using the Alexa Skills Kit SDK for Python.
# It is for sending commands to a robot and receiving replies.

# Please visit https://alexa.design/cookbook for additional examples on implementing slots, dialog management,
# session persistence, api calls, and more.
# This sample is built using the handler classes approach in skill builder.
import logging
import ask_sdk_core.utils as ask_utils

from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from ask_sdk_core.handler_input import HandlerInput

from ask_sdk_model import Response
from awscrt import mqtt
from awsiot import mqtt_connection_builder
import time

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class AWSMQTTPublisher(object):
    def __init__(self):

        # Parameters for AWS IoT MQTT Client.
        self.iotThingEndpoint = "<insert end point>"
        self.iotThingPort = 8883
        self.certificatePath = "certificates/"
        self.rootCAPath = self.certificatePath + "AmazonRootCA1.pem" # eg. "AmazonRootCA1.pem"
        self.privateKeyPath = self.certificatePath + "<your bot>.private.key" # eg. "VoiceControlledRobot.private.key"
        self.certificatePath = self.certificatePath + "<your bot>.cert.pem" # eg. "VoiceControlledRobot.cert.pem"
        self.commandTopic = "command_topic"
        self.responseTopic = "response_topic"
        self.clientID = "sdk-nodejs-publisher"
        self.mqtt_connection = None
        self.response = ""
        self._connected = False

        # Create a MQTT connection
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
            reconnect_min_timeout_secs=0,
            reconnect_max_timeout_secs=0,
            http_proxy_options=None,
            on_connection_success=self.on_connection_success,
            on_connection_failure=self.on_connection_failure,
            on_connection_closed=self.on_connection_closed)        

    def connect(self):
        # Connect and subscribe to AWS IoT
        logger.info("attemping to connect.")
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
        logger.info("Subscribing to topic '{}'...".format(self.responseTopic))
        subscribe_future, packet_id = self.mqtt_connection.subscribe(
            topic=self.responseTopic,
            qos=mqtt.QoS.AT_MOST_ONCE,
            callback=self.on_response_received)

        subscribe_result = subscribe_future.result()
        logger.info("Subscribed with {}".format(str(subscribe_result['qos'])))

    def disconnect(self):
        if self._connected:
            disconnect_future = self.mqtt_connection.disconnect()
            result = disconnect_future.result()
            print("disconnect result = ", result)
            self._connected = False
            logger.info("disconnected")
        
    def publish_command(self, command):
        if not self._connected:
            self.connect()
        if self._connected:
            self.response = ""
            publish_future, pub_id = self.mqtt_connection.publish(
                topic=self.commandTopic,
                payload=command,
                qos=mqtt.QoS.AT_MOST_ONCE)
            result = publish_future.result()
            print("publish result = ", result)
            logger.info("published message {} on topic {} id = {}".format(command, self.commandTopic, pub_id))
        else:
            logger.error("cannot publish, there is no active connection.")
                
    # Callback when connection is accidentally lost.
    def on_connection_interrupted(self, connection, error, **kwargs):
        logger.error("Connection interrupted. error: {}".format(error))
        self.disconnect()

    # Callback when an interrupted connection is re-established.
    def on_connection_resumed(self, connection, return_code, session_present, **kwargs):
        logger.info("Connection resumed. return_code: {} session_present: {}".format(return_code, session_present))
        
        if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
            logger.info("Session did not persist. Resubscribing to existing topics...")
            resubscribe_future, _ = connection.resubscribe_existing_topics()

            # Cannot synchronously wait for resubscribe result because we're on the connection's event-loop thread,
            # evaluate result with a callback instead.
            resubscribe_future.add_done_callback(self.on_resubscribe_complete)
        self._connected = True
    
    def on_resubscribe_complete(self, resubscribe_future):
        resubscribe_results = resubscribe_future.result()
        logger.info("Resubscribe results: {}".format(resubscribe_results))

        for topic, qos in resubscribe_results['topics']:
            if qos is None:
                logger.error("Server rejected resubscribe to topic: {}".format(topic))
    
    # Callback when the subscribed topic receives a message
    def on_response_received(self, topic, payload, dup, qos, retain, **kwargs):
        logger.info("Received response from topic '{}': {}".format(topic, payload))
        self.response = payload.decode()
        
    # Callback when the connection successfully connects
    def on_connection_success(self, connection, callback_data):
        assert isinstance(callback_data, mqtt.OnConnectionSuccessData)
        logger.info("Connection Successful with return code: {} session present: {}".format(callback_data.return_code, callback_data.session_present))
        self._connected = True
    
    # Callback when a connection attempt fails
    def on_connection_failure(self, connection, callback_data):
        assert isinstance(callback_data, mqtt.OnConnectionFailureData)
        logger.error("Connection failed with error code: {}".format(callback_data.error))
        self._connected = False
        
    # Callback when a connection has been disconnected or shutdown successfully
    def on_connection_closed(self, connection, callback_data):
        logger.info("Connection closed")
        self._connected = False
    
    # Wait for response on the response topic
    def waitForResponse(self):
        elapsed = 0.0
        while len(self.response) == 0:
            time.sleep(0.1)
            elapsed += 0.1
            if elapsed > 5:
                break
        if elapsed > 5:
            return "I'm not sure what happened. Try again."
        else:
            return self.response

# Open MQTT Connection
publisher = AWSMQTTPublisher()
publisher.connect()

class LaunchRequestHandler(AbstractRequestHandler):
    """Handler for Skill Launch."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return ask_utils.is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "Welcome, you can give Big Orange commands or ask him status information."

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )


class GotoIntentHandler(AbstractRequestHandler):
    """Handler for Go to Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("GotoIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        #speak_output = "ok"
        location = ask_utils.get_slot_value(handler_input, "location")
        publisher.publish_command("go to {}".format(location))
        speak_output = publisher.waitForResponse()
        publisher.disconnect()
        
        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class StatusIntentHandler(AbstractRequestHandler):
    """Handler for Status Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("StatusIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        statusType = ask_utils.get_slot_value(handler_input, "statusType")
        publisher.publish_command(statusType)
        speak_output = response = publisher.waitForResponse()
        publisher.disconnect()
        
        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class TakePictureIntentHandler(AbstractRequestHandler):
    """Handler for Take A Picture Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("TakePictureIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        #speak_output = "OK."
        publisher.publish_command("take a picture")
        speak_output = publisher.waitForResponse()
        publisher.disconnect()
        
        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class RechargeIntentHandler(AbstractRequestHandler):
    """Handler for Recharge Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("RechargeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        
        publisher.publish_command("go recharge")
        speak_output = publisher.waitForResponse()
        publisher.disconnect()
        
        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )

class WhereAreYouIntentHandler(AbstractRequestHandler):
    """Handler for Where Are You Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("WhereAreYouIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        publisher.publish_command("where are you")
        speak_output = publisher.waitForResponse()
        publisher.disconnect()

        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )

class StopAllMovementIntentHandler(AbstractRequestHandler):
    """Handler for Stop All Movement Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("StopAllMovementIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        #speak_output = "ok"
        response = publisher.publish_command("stop all movement")
        speak_output = publisher.waitForResponse()
        publisher.disconnect()
        
        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class HelpIntentHandler(AbstractRequestHandler):
    """Handler for Help Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("AMAZON.HelpIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "You can ask Big Orange for status information or give him commands. What would you like to do?"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )


class CancelOrStopIntentHandler(AbstractRequestHandler):
    """Single handler for Cancel and Stop Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return (ask_utils.is_intent_name("AMAZON.CancelIntent")(handler_input) or
                ask_utils.is_intent_name("AMAZON.StopIntent")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "Goodbye!"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .response
        )

class FallbackIntentHandler(AbstractRequestHandler):
    """Single handler for Fallback Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("AMAZON.FallbackIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        logger.info("In FallbackIntentHandler")
        speech = "Hmm, I'm not sure. You can ask Big Orange questions or tell him to do something."
        reprompt = "I didn't catch that. What can I help you with?"

        return handler_input.response_builder.speak(speech).ask(reprompt).response

class SessionEndedRequestHandler(AbstractRequestHandler):
    """Handler for Session End."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response

        # Any cleanup logic goes here.
        return handler_input.response_builder.response


class IntentReflectorHandler(AbstractRequestHandler):
    """The intent reflector is used for interaction model testing and debugging.
    It will simply repeat the intent the user said. You can create custom handlers
    for your intents by defining them above, then also adding them to the request
    handler chain below.
    """
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("IntentRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        intent_name = ask_utils.get_intent_name(handler_input)
        speak_output = "You just triggered " + intent_name + "."

        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class CatchAllExceptionHandler(AbstractExceptionHandler):
    """Generic error handling to capture any syntax or routing errors. If you receive an error
    stating the request handler chain is not found, you have not implemented a handler for
    the intent being invoked or included it in the skill builder below.
    """
    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        logger.error(exception, exc_info=True)

        speak_output = "Sorry, I had trouble doing what you asked. Please try again."

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

# The SkillBuilder object acts as the entry point for your skill, routing all request and response
# payloads to the handlers above. Make sure any new handlers or interceptors you've
# defined are included below. The order matters - they're processed top to bottom.


sb = SkillBuilder()

sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(GotoIntentHandler())
sb.add_request_handler(StatusIntentHandler())
sb.add_request_handler(TakePictureIntentHandler())
sb.add_request_handler(RechargeIntentHandler())
sb.add_request_handler(WhereAreYouIntentHandler())
sb.add_request_handler(StopAllMovementIntentHandler())
sb.add_request_handler(HelpIntentHandler())
sb.add_request_handler(CancelOrStopIntentHandler())
sb.add_request_handler(FallbackIntentHandler())
sb.add_request_handler(SessionEndedRequestHandler())
sb.add_request_handler(IntentReflectorHandler()) # make sure IntentReflectorHandler is last so it doesn't override your custom intent handlers

sb.add_exception_handler(CatchAllExceptionHandler())

lambda_handler = sb.lambda_handler()