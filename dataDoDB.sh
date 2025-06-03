#!/bin/bash


# Path to database
DB="/var/www/html/sensordata.db"


# Function to read serial data from USB port
readSerialData(){

  # Variables
	local line  # Store line
	local data_started=false  # Bool to know if data started
	local count=0  # Count of data
	local values=()  # Read data

 
# Read data line by line, set Internal Field Separator to none so the line is as is, -r so no escape characters, read and put value to line
	while IFS= read -r line; do
	if [[ "$line" == "new" ]]; then  # If new data arrived
		data_started=true
		count=0
		values=()
		continue
	fi
	
	if $data_started; then
		if [[ -z "$line" ]]; then  # Ignore blank lines
			continue
		fi
	
		values+=("$line")  # Add to values
		count=$((count+1))

    # If all data are present (13)
		if [[ "$count" -eq 13 ]]; then
			insert_data "$values[@]}" # Calling funtion to put values into SQL statement
			data_started=false
			sleep 0.1
		fi

	fi	
	
	done < <(cat < /dev/ttyACM0)  # Specification of input for loop
}


# Function to insert data
insert_data(){
  # SQL statement that doesnt change
	sql="INSERT INTO data(datum, cas, intenzitaS, vstupVI, bocniVI, zatezVI, proudI, vstupV1, bocniV1, zatezV1, proud1, vstupV2, bocniV2, zatezV2, proud2) VALUES (CURRENT_DATE, CURRENT_TIME,"

  # Add data to SQL statement
	for value in "$@"; do
		sql+="$value,"
	done

  # Trim last , from string
	sql="${sql%,})"

  # Execute sql statement
	sqlite3 $DB "$sql"
}

# Call function to read Data
readSerialData
