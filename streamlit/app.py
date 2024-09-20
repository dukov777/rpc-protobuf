import streamlit as st
import serial.tools.list_ports
import serial
import encoder_serial

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# st.title("Calibrate the board")

ports = list_serial_ports()

# Using "with" notation
with st.sidebar:
    if ports:
        global selected_port
        selected_port = st.selectbox("Select a Serial Port", ports)
        global boudrate
        boudrate = st.radio(
            "Choose a boudrate",
            ("115200", "9600")
        )        
    else:
        st.write("No serial ports found.")

if st.button("Send 1"):
    try:
        print(selected_port, boudrate)
        encoder_serial.process_serial_message(selected_port, boudrate, 1, 1)
    except serial.SerialException as e:
        st.write(e)
    except encoder_serial.TransportError as e:
        st.write(e)
    except encoder_serial.ProtocolError as e:
        st.write(e)
    except Exception as e:
        st.write(e)
 
