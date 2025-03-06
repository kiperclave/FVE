import serial
import sqlite3
import datetime

cas = datetime.datetime.now();

conn = sqlite3.connect('/sensordata.db')

cursor = conn.cursor()

vstup = []

poziceVstup = 0;

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            line = ser.readLine().decode('utf-8').rstrip()
            if(line == "new"):
                vstup = null
                poziceVstup = 0
                pomocny = ""
            else:
                vstup.append(line)

        if(poziceVstup >= 17):
            pomocny = f'insert into data values (\'{x.date()}\', \'{x.strftime("%X")}\''
            for x in range(17):
                pomocny += f",\'{vstup[x]}\'"

            pomocny += ")"
            cursor.execute(pomocny)

            print("Working!")
