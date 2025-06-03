# import pandas as pd

# complete_data = pd.DataFrame()

def convert( log_filename ) :

    csv_file_data = ""

    with open( log_filename , "r" ) as _file :
        file_data = _file.readlines()
        print(len(file_data))
        for line in file_data :
            time_data,actual_data = line.split("- INFO -")
            if actual_data.strip().startswith("Obstacle distance:") :
                final_data = actual_data.split("Obstacle distance:")[1].split(" - ")
                csv_file_data += final_data[2]+","+final_data[3]+","+final_data[4]+","+final_data[5]+","+final_data[1].replace("[","").replace("]","").replace("\n","")+",\n"
                # print(final_data)

    with open( log_filename.replace(".log",".csv") , "w" ) as _file :
        _file.write(csv_file_data)

log_filename = "/home/amit-singh/Downloads/qudacopter/20250530175657_offboard_control.log"
convert( log_filename ) 
