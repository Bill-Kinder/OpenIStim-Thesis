# Bill Kinder 31-10-2024
# Bachelor of Electrical Engineering (Honours), The University of Sydney.
# OpenIStim Graphical User Interface
# This Python program connects to the internet to send messages via an MQTT Broker. The Broker is hard coded but can be modified in this script.
# This program is designed to send/receive messages to/from an Arduino controlled, constant-current spinal cord stimulation device (https://github.com/MonzurulAlam/OpenIstim)

# Note: Press ALT+Z to wrap comment text aross lines.

# Libraries - TKINTER, MQTT, JSON, TIME and MATH (Trunc)

import tkinter as tk
import paho.mqtt.client as mqtt
import json
from time import time
from math import trunc

#############
#           #
# FUNCTIONS #
#           #
#############

# Functions section: Every function used in the GUI and the MQTT client is defined in this section.

def on_connect(client, userdata, flags, rc, x):
    # MQTT Callback function.
    # This function defines what happens after the MQTT client has connected to the MQTT broker.
    # rc == 0 --> Successful connection.
    # Overview: Subscribe to the relevant topics, print results to the terminal. Modify the connection button (GUI) to display "Disconnect from Broker" instead of "Connect to Broker". Ping the Arduino.

    if rc==0:
        # Define references to global variables, such as the topics, labels, buttons and flags.
        global topic_data, topic_test_gui, topic_ping_gui, topic_feedback, topic_params_gui, qos, is_connected_label, is_connected, toggle_connection_button, prev_ping_time

        # Subscribe to topics: These topics have messages sent to them from Arduino.
        client.subscribe(topic_data, qos)
        client.subscribe(topic_test_gui, qos)
        client.subscribe(topic_ping_gui, qos)
        client.subscribe(topic_feedback, qos)
        client.subscribe(topic_params_gui, qos)

        # Print a "connection OK" message.
        print(f"Connected OK\nReturned code = {rc}")
        sysMsgDisplay(msg = f"Connected OK\nReturned code = {rc}")

        # Change the connection button (invert the functionality).
        toggle_connection_button.config(text = "Disconnect from Broker")

        # Set connection flag to true and change the text to say "Client is connected to Broker".
        is_connected = True
        is_connected_label.config(text = f"Client is connected to Broker", fg = "green")

        # Send a ping to the Arduino.
        sendPing()

        # Start counting time between pings from Arduino
        prev_ping_time = time()
        updateArduinoPingLabel()

    else: # Connection failed.
        # Print error message.
        print(f"Bad connection\nReturned code = {rc}")
        sysMsgDisplay(msg = f"Bad connection\nReturned code = {rc}")
    # End
    return

def on_disconnect(client, userdata, flags, rc, x):
    # MQTT Callback function.
    # This function defines what happens after the MQTT client has disconnected from the MQTT broker.
    # This will occur when the user clicks the "Disconnect from Broker" button on the GUI.
    global is_connected, toggle_connection_button, is_connected_label, sys_msg_label

    # Print the reason code for disconnection (e.g.: Normal disconnection, unspecified error, etc). If there are problems with connection this reason code can be used to debug.
    print(f"Disconnected\nReturned code = {rc}")
    sysMsgDisplay(msg = f"Disconnected\nReturned code = {rc}")
    # Change the connection button (invert the functionality).
    toggle_connection_button.config(text = "Connect to Broker", bg = "light green")

    # Set the connection flag to false, change the connection label to say "Client is not connected to Broker".

    is_connected = False
    is_connected_label.config(text = f"Client is not connected to Broker", fg = "Black")

    # End
    return

def on_publish(client, userdata, mid, rc, x):
    # MQTT Callback function. 
    # This function defines what happens after the MQTT client has successfully published a message.
    # rc == 0 --> Successfully published the message.

    if(rc == 0):
        # Print success message to the terminal.
        print("Message published to mqtt broker successfully, returned code = ", rc)

    else:
        # Print error message to the terminal.
        print("For some reason, the message could not be published. Check your internet and MQTT connection.")

    # End
    return

def on_message(client, userdata, msg):
    # MQTT Callback function. 
    # This function defines what happens when the MQTT client receives a message.
    # Overview: The message is referenced by the parameter "msg" in the function definition. The message payload and topic are stored in temporary variables and printed to the terminal. Then, the topic is checked, and a decision is made based on the topic. An overview of each topic is found under each IF statement.
    
    global sys_msg_label, toggle_stimulation_button, topic_data, topic_ping_gui, topic_test_gui, topic_feedback, topic_params_gui, send_time, recv_time, latency, latency_msg, prev_ping_time, incoming_ping_count, new_ping_time

    # Store payload and topic in local variables.    
    msg_payload = msg.payload.decode('utf-8')
    msg_topic = msg.topic

    # Print the message payload, topic and qos.
    print('Received new message. Contents:\n', str(msg_payload))
    print('message topic =', msg_topic)
    print('message qos=', msg.qos)

    # If the topic is "gui/data", the data is extracted as a json dictionary from the payload, then the function "dispData()" is called, which displays the data on the GUI.
    if(msg_topic == topic_data):
        global data
        data = json.loads(msg_payload)
        dispData()
        if(not data["currently_stimulating"]):
            toggle_stimulation_button.config(text = "Start Stimulation")
        else:
            toggle_stimulation_button.config(text = "Stop Stimulation")
    
    # If the topic is "gui/ping", the ping count is updated on the GUI. The purpose of this is to let the user know if the Arduino is still connected to the Broker. If the number is changing, the Arduino is pinging and thus is connected to the Broker. Otherwise, the Arduino is disconnected from the Broker (or the Python client). The ping is programmed to occur once every 5 seconds.
    elif(msg_topic == topic_ping_gui):
        incoming_ping_count = msg_payload
        prev_ping_time = time()

    # If the topic is "gui/test", the test message acknowledgement count is updated on the GUI. This is essentially the same as "gui/ping" except it is not a timed ping, rather it is feedback from pressing the "Send Test Message" button.    
    elif(msg_topic == topic_test_gui):
        recv_time = time()
        latency = round(recv_time - send_time, 3)
        latency_msg = f"\nLatency {latency}s"
        test_msg_label.config(text = f"Test message acknowledgments: {msg_payload}" + latency_msg)
    
    # If the topic is "gui/feedback", the message is displayed in the "system messages" panel. This feedback topic specifically relates to whether the parameters are successfully sent.
    elif(msg_topic == topic_feedback):
        recv_time = time()
        latency = round(recv_time - send_time, 3)
        latency_msg = f"\nLatency {latency}s"
        sysMsgDisplay((msg_payload + latency_msg))

    # If the topic is "gui/params", the current parameters in the parameters boxes are erased and replaced with the current value of the parameters in the Arduino program.
    elif(msg_topic == topic_params_gui):
        params = json.loads(msg_payload)
        recv_time = time()
        latency = round(recv_time - send_time, 3)
        latency_msg = f"\nLatency {latency}s"
        sysMsgDisplay("Parameters received from Arduino." + latency_msg)
        dispParams(params)
    
    # If the topic is none of the above, nothing is done.
    else:
        print('No specific topic instructions found.')

    # End
    return

def sendTestMsg(event):
    # The purpose of this function is to send a test message to the Arduino. This function is activated when the "Send Test Message" button is clicked.
    global myClient, qos, retain, topic_test_stim, send_time
    myClient.publish(topic = topic_test_stim, payload = "This is a test!", qos = qos, retain = retain)
    send_time = time()
    # End
    return

def sysMsgDisplay(msg):
    # This function displays a message on the "System Messages" sub-panel.
    sys_msg_label.config(text = msg)
    
    # End
    return

def modifyParamEntry(index, new_val):
    # This function deletes what's in the current parameter entry box and fills it in with a new value. This function is called when the frequency parameter needs to be adjusted and when getting parameters from Arduino.
    # index: parameter index.
    # new_val: Value to replace the current text.
    param_entries[index].delete(0, tk.END) # Delete what is currently there.
    param_entries[index].insert(0, new_val) # Insert the new value.

    # End
    return

def setStimParams(event):
    # This function reads the stimulation parameters entered into the GUI text boxes, then sends them to the Broker, to be received by Arduino.
    # If every parameter is valid, store the parameters in a JSON format, then send the JSON data to the broker using the client publish command. Send under the topic "stim/params". Wait for a feedback signal from the broker. Display the feedback message in the system messages frame (Success/Failure due to incorrect password).
    # If the parameters are somehow invalid, a message will be displayed in system messages to notify the user.
    global myClient, qos, retain, is_connected, param_entries, param_labels, topic_params_stim, send_time

    # Initialise the message to be displayed in the system messages panel.
    result_string = ""

    # Initialise the send parameters flag.
    send_the_params = True

    # Check if the values are valid by trying to convert them to floats.
    try:
        pf_temp = float(param_entries[0].get()) # Pulse Frequency (Hz)
        pw_temp = float(param_entries[1].get()) # Pulse Width (µs)
        dur_temp = float(param_entries[2].get())# Duration (s)
        intensity_temp =  float(param_entries[3].get()) # Intensity
        calibration_temp = float(param_entries[4].get())# Calibration
        pb_temp = float(param_entries[5].get()) # Pulses per burst
    except ValueError:
        # Display system error message.
        sysMsgDisplay("ValueError: Parameter values are not valid. Each parameter must be given a numerical value.\nNot sending parameters.")
        # End
        return
    
    # Check duration. Set to 20 if not positive.
    if(dur_temp <= 0):
        modifyParamEntry(2, 20)
        result_string += "Duration must be a positive number. Changing to 20 seconds.\n"
    
    # Check pulse frequency, pulse width, pulses per burst, intensity and calibration. If any of them are not positive, display an error message, stop the parameters from being sent.
    if(pf_temp <= 0 or pw_temp <= 0 or pb_temp <= 0 or intensity_temp <= 0 or calibration_temp <= 0):
        result_string += "Invalid input: Variables must be positive numbers.\n"
        send_the_params = False

    # If all the total pulse bursts exceed the pulse period (1/pulse frequency), replace the pulse frequency with a value that can contain the total number of pulse bursts, effectively lengthening the output signal and allowing the pulse bursts to "fit" in each cycle.
    elif (pw_temp*pb_temp*2 > (1e6/pf_temp)):
        # Calculate period.
        new_carrier_period = pw_temp*pb_temp*2.1

        # Calculate frequency.
        new_pulse_frequency = trunc(1e6/new_carrier_period)

        # If frequency is valid, replace the frequency and output a message.
        if(new_pulse_frequency > 0):
            result_string += "Pulse duration exceeds period, decreasing pulse frequency.\n"
            modifyParamEntry(0, new_pulse_frequency)
        else: #If frequency is 0, prevent the parameters from sending, output an error message.
            result_string += "Invalid input: Pulse width and Pulses per Burst too large to account for Pulse Frequency.\n"
            send_the_params = False

    # If not connected to the Broker, prevent the parameters from sending, output an error message.
    if(not(is_connected)):
        result_string += "Not connected to the broker.\n"
        send_the_params = False
    
    # If the parameters are valid after above checks, initialise an empty dictionary, fill the dictionary using the parameter entries.
    if(send_the_params):
        send_parameters = dict()
        # Get each parameter one by one.
        for i in range(0, len(param_entries)):
            key = param_labels[i].cget("text")
            value = param_entries[i].get()
            send_parameters[key] = value
        # Publish the parameters to the Broker, output a message.
        myClient.publish(topic = topic_params_stim, payload = json.dumps(send_parameters), qos = qos, retain = retain)
        send_time = time()
        result_string += "Parameters published to MQTT Broker."
    else: # Parameters invalid or client not connected.
        result_string += "Parameters not sent."
    # Display the total message in the system messages frame.
    sysMsgDisplay(result_string)

    # End
    return

def getStimParams(event):
    # This function sends a command to Arduino to print its parameter values to the GUI. The command is published under the topic "stim/cmd". This function is called when the "Get Parameters" button is pressed.
    global myClient, qos, retain, is_connected, topic_cmd, send_time
    # Check connection status. If connected, publish the command and display a message. If not, display an error message.
    if(is_connected):
        myClient.publish(topic = topic_cmd, payload = "get parameters", qos = qos, retain = retain)
        send_time = time()
        sysMsgDisplay("Requesting parameters from Arduino.")
    else:
        sysMsgDisplay("Command failed: Not connected to the MQTT Broker.")

    # End
    return

def connectMQTT():
    # This function connects the client to the MQTT broker. It is called when the "Connect to Broker" button is clicked.
    # myClient: The client.
    # broker: The MQTT Broker address.
    # port: The MQTT Broker port.
    global myClient, broker, port
    myClient.connect(broker, port) # Connect the client to the Broker.
    myClient.loop_start() # Start the client loop - this "activates" the client i.e.: It begins listening for messages and will publish messages when it is programmed to do so.

    # End
    return

def disconnectMQTT():
    # This function disconnects the client from the MQTT broker. It is called when the "Disconnect from Broker" button is clicked.
    global myClient
    myClient.disconnect()

    # End
    return

def connectionButton(event):
    # This function determines whether clicking the toggle_connection_button (GUI element) connects or disconnects the client and Broker. It is activated when the toggle_connection_button is clicked (either Connect to Broker or Disconnect from Broker.)
    global is_connected
    if(is_connected):
        disconnectMQTT()
    else:
        connectMQTT()

    # End.
    return

def stimulationButton(event):
    # This function starts or stops the stimulation, depending on whether the stimulator is currently outputting the stimulation pulses. Does nothing if the client is not connected.
    global myClient, qos, retain, is_connected, topic_cmd, data, toggle_stimulation_button, send_time
    
    # If not connected to MQTT Broker, display an error message.
    if(not(is_connected)): 
        sysMsgDisplay("Command failed: Not connected to the MQTT Broker.")
        return
    
    # Stimulation ON: Send a command to stop the stimulation, display a system message.
    if(data["currently_stimulating"]):
        myClient.publish(topic = topic_cmd, payload = "stop stimulation", qos = qos, retain = retain)
        send_time = time()
        sysMsgDisplay("Command sent to stop stimulation.")
        
    # Stimulation OFF: Send a command to start the stimulation, display a system message.
    else: 
        myClient.publish(topic = topic_cmd, payload = "start stimulation", qos = qos, retain = retain)
        send_time = time()
        sysMsgDisplay("Command sent to start stimulation.")

    # End
    return

def drawParamBoxes():
    # This function arranges each parameter entry box, label and frame so the text boxes are organised neatly in the window.
    # stim_parameters: Dictionary of stimulation parameters
    # param_entries: Empty array to store Tkinter Entry widgets.
    # param_frame: Empty array to store Tkinter labels.
    global stim_parameters, param_entries, param_frame

    # Determine maximum allowable width + ensure each parameter entry box is the same width.
    maxWidth = max(len(key) for key in stim_parameters.keys())

    # Iterate through each parameter. Draw the frame, label and entry widgets, add them to the relevant arrays.
    for key in stim_parameters:
        # Frame
        entry_sub_frame = tk.Frame(
            master = param_frame,
            relief = tk.RIDGE,
            borderwidth = 5,
            width = maxWidth*2
        )
        # Label
        label = tk.Label(
            master = entry_sub_frame,
            text = key,
            width = maxWidth,
            height = 1
        )
        # Entry box
        entry = tk.Entry(
            master = entry_sub_frame,
            text = tk.StringVar(),
            width = maxWidth
        )
        # Append to arrays
        param_labels.append(label)
        param_entries.append(entry)
        label.grid(sticky = "nsew")
        entry.grid(sticky = "nsew")
        entry_sub_frame.grid()
        # End of FOR-LOOP
    
    # password_label and password_entry: The same as parameters, but specifically for the password, which is treated as a "parameter" for the purposes of message publishing, but is only used to verify a parameter change on the Arduino side.
    global password_label, password_entry
    # Frame
    password_sub_frame = tk.Frame(
        master = param_frame,
        relief = tk.RIDGE,
        borderwidth = 5,
        width = maxWidth*2
    )
    # Label
    password_label = tk.Label(
        master = password_sub_frame,
        text = "Password",
        font = "TKDefaultFont 12 bold",
        width = maxWidth,
        height = 1
    )
    # Entry box
    password_entry = tk.Entry(
        master = password_sub_frame,
        text = tk.StringVar(),
        width = maxWidth
    )
    password_label.grid()
    password_entry.grid()
    password_sub_frame.grid()
    param_labels.append(password_label)
    param_entries.append(password_entry)

    # End
    return 

def sendPing():
    # This function sends a ping message to the arduino under the topic "stim/ping". It repeats once per second if the client is connected the Broker. Message: "Hello Arduino, from Python. Ping count: x". This function is first executed upon connection to the Broker and is repeated once per second.
    global is_connected, gui_ping_label
    if(is_connected):
        global ping, myClient, qos, retain, topic_ping_stim
        myClient.publish(topic = topic_ping_stim, payload = f"Hello Arduino, from Python.\nPing count:\n{ping}", qos = qos, retain = retain)
        ping += 1
        gui_ping_label.config(text = f"Ping to Arduino count: {ping}")
        # Repeat after 1000 milliseconds.
        main_window.after(1000, sendPing)
    
    # End
    return

def updateArduinoPingLabel():
    # This function updates the Arduino ping label to display how long it has been since the GUI received a ping message from the Arduino. It is called once every second.
    global prev_ping_time
    arduino_ping_label.config(text = f"Arduino Ping Count: {incoming_ping_count}\nLast received: {int(time() - prev_ping_time)}s ago")
    main_window.after(1000, updateArduinoPingLabel)

def dispData():
    # This function displays data sent from Arduino to the GUI via the "gui/data" topic.
    # data_labels_dict: A dictionary containing each data variable name to be displayed on screen.
    # data: The numerical values of the variables in data_labels_dict
    global data, data_labels_dict
    # Iterate through each label.
    for key in data_labels_dict:
        data_labels_dict[key].config(text = f"{key}: {data[key]}")
    
    # End
    return

def dispParams(params):
    # This function displays the parameters in the parameter boxes, after the "Get parameters" button is clicked and the client receives the parameters currently stored in Arduino.
    # params: the parameters data received from Arduino.
    # param_entries: an array of Tkinter Entry widgets representing each parameter.
    global param_entries

    # Break up parameters into an array key-value tuples. Temporary variable.
    list_of_params = list(params.items())

    # For each parameter entry widget, replace the current value with the value from `list_of_params`
    for i in range(0, 6):
        param_entries[i].delete(0, tk.END)
        param_entries[i].insert(0, list_of_params[i][1])
    return

####################
#                  #
#  MQTT CONSTANTS  #
#                  #
####################

# Broker Address and Port

broker = 'test.mosquitto.org'
port = 1883

# Topics: From Stimulator, to GUI

topic_data = "gui/data"
topic_test_gui = "gui/test"
topic_ping_gui = "gui/ping"
topic_feedback = "gui/feedback"
topic_params_gui = "gui/params"

# Topics: From GUI, to Stimulator

topic_params_stim = "stim/params"
topic_test_stim = "stim/test"
topic_ping_stim = "stim/ping"
topic_cmd = "stim/cmd"

# Quality of Service, Retain and Connection Flags.

qos = 1
retain = True
is_connected = False

# Ping variable (sent to Arduino).

ping = 0

# Ping timing variables (received from Arduino).

prev_ping_time = 0
incoming_ping_count = 0

# Timing variable (Latency)

send_time = 0
recv_time = 0
latency = 0
latency_msg = "Latency: N/A"

# MQTT Client Object initialisation, assigning callback functions.

myClient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
myClient.on_message = on_message
myClient.on_connect = on_connect
myClient.on_publish = on_publish
myClient.on_disconnect = on_disconnect

###################
#                 #
#  TKINTER & GUI  #
#                 #
###################

# TKinter Window Object

main_window = tk.Tk()
main_window.title("OpenIStim Interface")

# Dimension Constants

buttonWidth = 25
labelWidth = 35
frameWidth = 50

# Stimulation parameters dictionary initialisation and example values for each parameter.

stim_parameters = dict()
stim_parameters["Pulse Frequency (Hz)"] = 20
stim_parameters["Pulse Width (µs)"] = 50
stim_parameters["Duration (s)"] = 450
stim_parameters["Intensity"] = 0
stim_parameters["Calibration"] = 40
stim_parameters["Pulses per Burst"] = 10

######################

# DATA FRAME OBJECTS #

######################

# Background colour

colour_data_frame_bg = "lightblue2"

# Data frame object (rightmost frame in the window)

data_frame = tk.Frame(
    master = main_window,
    relief = tk.GROOVE,
    borderwidth = 5,
    width = frameWidth,
    bg = colour_data_frame_bg
)

# Data frame title label.

data_title_label = tk.Label(
    master = data_frame,
    text = "Data",
    font = "TKDefaultFont 12 bold",
    width = labelWidth,
    height = 5,
    bg = colour_data_frame_bg
)

# Grid the label --> Draw the label within the frame.

data_title_label.grid()

# Expected attributes from data retrieval, stored in dictionary format. Initialised with arbitrary values.

expected_data_dict = {"Current (mA)": 0,
                      "v1" : 0,
                      "v2" : 0,
                      "currently_stimulating" : False,
                      "timer_value": 0,
                      "num_cycles" : 0}

# Data variable - updated when requesting data from Arduino. The format sent by Arduino slightly differ

data = expected_data_dict

# Initialise empty array to hold data labels (i.e.: one label object for each of "Current (mA)", "v1", etc.)

data_labels = []

# Iterate over each possible data (current, v1, v2, etc) and create a label for it.

for key in expected_data_dict:
    data_labels.append(tk.Label(
        master = data_frame,
        text = f"{key}: {expected_data_dict[key]}",
        borderwidth = 5,
        width = 25,
        bg = colour_data_frame_bg
        )
    )

# Data labels dictionary - this matches the order sent by Arduino.
    
data_labels_dict = {"Current (mA)": data_labels[1],
                      "v1" : data_labels[2],
                      "v2" : data_labels[3],
                      "currently_stimulating" : data_labels[4],
                      "timer_value" : data_labels[0],
                      "num_cycles" : data_labels[5]}

# Draw each data label in the label frame.

for label in data_labels:
    label.grid()

gui_ping_label = tk.Label(
    master = data_frame,
    text = f"Ping to Arduino count: {ping}",
    width = labelWidth,
    height = 5,
    bg = colour_data_frame_bg
)

gui_ping_label.grid()

####################

# PARAMETERS FRAME #

####################

# Background colour

colour_param_frame_bg = "lightcyan1"

# Empty arrays for TK entry and label objects

param_entries = []
param_labels = []

# Parameter frame object (middle frame in the application window)

param_frame = tk.Frame(
    master = main_window,
    relief = tk.GROOVE,
    borderwidth = 5,
    width = frameWidth,
    bg = colour_param_frame_bg
)

# Parameter frame title label.

param_title_label = tk.Label(
    master = param_frame,
    text = "Parameters",
    font = "TKDefaultFont 12 bold",
    width = labelWidth,
    height = 5,
    bg = colour_param_frame_bg
)

# Send parameter button

send_params_button = tk.Button(
    text = "Set Parameters",
    width = buttonWidth,
    height = 2,
    master = param_frame,
    highlightbackground=colour_param_frame_bg   
)

# Get parameter button.

get_params_button = tk.Button(
    text = "Get Parameters",
    width = buttonWidth,
    height = 2,
    master = param_frame,
    highlightbackground=colour_param_frame_bg   
)

# Toggle stimulation button (start/stop)

toggle_stimulation_button = tk.Button(
    text = "Start Stimulation",
    width = buttonWidth,
    height = 2,
    master = param_frame,
    highlightbackground=colour_param_frame_bg
)

# Draw the parameter frame title label into the parameter frame.
param_title_label.grid()

# Draw the parameter entry boxes and labels.
drawParamBoxes()

# Draw the buttons, bind the functions to the mouse left click.
# Send
send_params_button.grid()
send_params_button.bind("<Button-1>", setStimParams)
# Get
get_params_button.grid()
get_params_button.bind("<Button-1>", getStimParams)
# Toggle
toggle_stimulation_button.grid()
toggle_stimulation_button.bind("<Button-1>", stimulationButton)

####################

# CONNECTION FRAME #

####################

# Background colour

colour_connection_frame_bg = "lightblue2"

# Connection frame object (leftmost frame in the application window).

connection_frame = tk.Frame(
    master = main_window,
    relief = tk.GROOVE,
    borderwidth = 5,
    width = frameWidth,
    bg = colour_connection_frame_bg
)

# Connection frame title label.

connection_title_label = tk.Label(
    master = connection_frame,
    text = "Connection",
    font = "TKDefaultFont 12 bold",
    width = labelWidth,
    height = 5,
    bg = colour_connection_frame_bg
)

# System messages subframe - to be inside the connection frame.

sys_msg_subframe = tk.Frame(
    master = connection_frame,
    relief = tk.RIDGE,
    borderwidth = 5,
    width = labelWidth,
    bg = colour_param_frame_bg,
    height = 25
)

# System messages subframe title label.

sys_msg_title_label = tk.Label(
    master = sys_msg_subframe,
    text = "System Messages",
    font = "TKDefaultFont 12 bold",
    bg = colour_param_frame_bg
)

# System messages label - to display the received message inside the system messages subframe.

sys_msg_label = tk.Label(
    width = labelWidth,
    height = 20,
    master = sys_msg_subframe,
    wraplength = 300,
    bg = colour_param_frame_bg
)

# Toggle connection button object (connect/disconnect).

toggle_connection_button = tk.Button(
    text = "Connect to Broker",
    width = buttonWidth,
    height = 2,
    master = connection_frame,
    highlightbackground = colour_connection_frame_bg
)

# Connection flag label.

is_connected_label = tk.Label(
    text = f"Client is not connected to Broker",
    width = labelWidth,
    height = 2,
    master = connection_frame,
    bg = colour_connection_frame_bg
)

# Arduino ping label - display total number of pings received from Arduino since connnecting.

arduino_ping_label = tk.Label(
    text = "Arduino Ping Count: N/A\nLast received: N/A",
    width = labelWidth,
    height = 2,
    master = connection_frame,
    bg = colour_connection_frame_bg
)

# Test message button.

test_msg_button = tk.Button(
    text = "Send Test Message",
    width = buttonWidth,
    height = 2,
    master = connection_frame,
    highlightbackground = colour_connection_frame_bg
)

# Test message acknowledgement label.

test_msg_label = tk.Label(
    text = "Test message acknowledgments: N/A\nLatency: N/A",
    width = labelWidth,
    height = 2,
    master = connection_frame,
    bg = colour_connection_frame_bg
)

# Drawing each label and button to the frame. The order matters - objects gridded earlier are placed further to the top.

connection_title_label.grid()
is_connected_label.grid()
toggle_connection_button.grid()
toggle_connection_button.bind("<Button-1>", connectionButton)
arduino_ping_label.grid()
sys_msg_title_label.grid()
sys_msg_label.grid()
sys_msg_subframe.grid()
test_msg_label.grid()
test_msg_button.grid()
test_msg_button.bind("<Button-1>", sendTestMsg)

#######################

# GRIDDING THE FRAMES #

#######################

# The 3 major frames - connection (left), parameters (middle) and data (right). Each frame will contain its subframes, labels, entries and buttons as specified above.

connection_frame.grid(row = 0, column = 0, sticky = 'nsew')
param_frame.grid(row = 0, column = 1, sticky = 'nsew')
data_frame.grid(row = 0, column = 2, sticky = 'nsew')

# Begin the GUI window loop - this will allow the user to begin and control the MQTT client loop by interacting with the buttons and entries.

main_window.mainloop()

# That's all!