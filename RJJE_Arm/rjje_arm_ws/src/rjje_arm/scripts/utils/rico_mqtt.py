import paho.mqtt.client as mqtt
from typing import List, Dict, Callable

class MqttSubscriberCbs:
    @staticmethod
    def simple_decode(userdata, msg):
        return str(msg.payload.decode("utf-8"))
    @staticmethod
    def get_array_from_string(nums: str):
        ret = []
        for num in nums:
            try:
                ret.append(float(num))
            except ValueError:
                pass
        return ret

    @staticmethod
    def get_numerical_array(userdata, msg, delim : str =";"):
        """
        Assume a numerical array separated by a delimiter is passed in, e.g., [1.0; 2.0;]

        Args:
            userdata (_type_): private user-stored data
            msg (paho.mqtt.client.MQTTMessage): message
            delim (str, optional): delimiter
        """
        nums = str(msg.payload.decode("utf-8")).split(delim)
        return MqttSubscriberCbs.get_array_from_string(nums)

class MqttClient:
    def __init__(self, broker_ip:str, 
                 broker_port:int, 
                 subscriber_cbs: Dict[str, Callable[[int, bytes], None]], 
                 client_name:str = ""):

        def on_connect(client, userdata, flags, rc):
            subscribed_topics = list(subscriber_cbs.keys())
            for topic in subscribed_topics:
                client.subscribe(topic)  # Subscribe to the topic “digitest/test1”, receive any messages published on it
            print(f"{client_name} subscribes to {subscribed_topics}, result code {0}".format(str(rc)))  # Print result of connection attempt
        
        def on_message(client, userdata, msg):  
            # Signature of subscriber callbacks
            subscriber_cbs[msg.topic](userdata, msg)
        self.client = mqtt.Client(client_name)
        self.client.on_connect = on_connect
        self.client.on_message = on_message  
        self.client.connect(broker_ip, broker_port)
        # Start networking daemon
        self.client.loop_start()  
    
    def publish(self, topic, msg):
        self.client.publish(topic, msg)

if __name__ == "__main__":
    """
    This is a small test of the above functions 
    """
    def topic1_cb(userdata, msg):
        str = MqttSubscriberCbs.simple_decode(userdata, msg)
        #TODO 
        print(str)
    def topic2_cb(userdata, msg):
        arr = MqttSubscriberCbs.get_numerical_array(userdata, msg)
        print(arr)

    c = MqttClient("127.0.0.1", 1883,
                   {"test_topic1": topic1_cb, 
                    "test_topic2": topic2_cb}, 
                   "hello")
    import time
    for _ in range(10):
        c.publish("test_topic3", "merhaba")
        print("publishing")
        time.sleep(1)
    print("publishing done")
    