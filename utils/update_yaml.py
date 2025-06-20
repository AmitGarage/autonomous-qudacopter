import yaml
import sys

def update(central_yaml_file):
    # Load central parameters
    with open(central_yaml_file, "r") as f:
        central_data = yaml.safe_load(f)
    
    central_data_keys = central_data.keys()
    for central_data_key in central_data_keys:
        if central_data_key == "slam_toolbox" :
            central_data_key_file = "ros2_ws/src/"+central_data_key+"/config/mapper_params_online_async.yaml"
        else:
            central_data_key_file = "ros2_ws/src/"+central_data_key+"/config/params.yaml"
        
        # print(f"file_path - {central_data_key_file}")
        with open(central_data_key_file, "r") as f:
            actual_central_key_data = yaml.safe_load(f)
        
        # print("actual_central_key_data - ",actual_central_key_data)
        node_names = central_data[central_data_key]
        for node_name in node_names.keys() :
            parameter_names = node_names[node_name]
            for parameter_name in parameter_names.keys() :
                # print(node_name," - ",parameter_names[parameter_name])
                actual_central_key_data[node_name]['ros__parameters'][parameter_name] = parameter_names[parameter_name]

        with open(central_data_key_file, "w") as f:
            yaml.dump(actual_central_key_data, f, sort_keys=False)

        # print("actual_central_key_data - ",actual_central_key_data)
        print(f"'✅' {central_data_key_file} updated")
    print(f"\n📁 Finished.")

central_yaml = sys.argv[1]

update(central_yaml)