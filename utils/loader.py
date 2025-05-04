"""
Utility functions for loading robot configurations.
"""

import os
import cv2
import numpy as np

def load_configuration(config_file):
    """Load robot configuration from file."""
    try:
        data = np.loadtxt(config_file)
        if len(data) == 7:  # Position (3) + Quaternion (4)
            position = data[0:3].reshape(3, 1)
            quaternion = data[3:7]
            return {
                'position': position,
                'quaternion': quaternion
            }
        else:
            print(f"Invalid configuration file format: {config_file}")
            return None
    except Exception as e:
        print(f"Error loading configuration: {str(e)}")
        return None

def load_all_configurations(yaml_file=None):
    """Load all robot configurations from YAML file."""
    if yaml_file is None:
        yaml_file = "configurations.yaml"
        
    if not os.path.exists(yaml_file):
        print(f"Configuration file not found: {yaml_file}")
        return []
        
    try:
        # Try to detect file format - old format uses OpenCV FileStorage
        # New format uses Python YAML with nested structure
        with open(yaml_file, 'r') as f:
            content = f.read()
            
        # Check if this is the new format with metadata/configurations sections
        if "metadata:" in content and "configurations:" in content:
            print(f"Detected new YAML format in {yaml_file}")
            try:
                import yaml
                # First, clean the file content by removing problematic YAML directives
                cleaned_content = '\n'.join([line for line in content.split('\n') 
                                          if not line.startswith('%YAML') and not line == '---'])
                
                # Use safe_load on the cleaned content
                data = yaml.safe_load(cleaned_content)
                
                configs = []
                if data and 'configurations' in data:
                    for item in data['configurations']:
                        if 'position' in item and len(item['position']) == 3:
                            # Convert position list to numpy array with proper shape
                            position = np.array(item['position']).reshape(3, 1)
                            
                            # Handle missing quaternion by using a default value
                            quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Default identity quaternion
                            if 'quaternion' in item and len(item['quaternion']) == 4:
                                quaternion = np.array(item['quaternion'])
                                
                            configs.append({
                                'number': item.get('id', len(configs) + 1),
                                'position': position,
                                'quaternion': quaternion,
                                'image_file': item.get('image', '')
                            })
                    
                print(f"Loaded {len(configs)} configurations from new format YAML")
                return configs
            except Exception as yaml_error:
                print(f"Error parsing new YAML format: {str(yaml_error)}")
                print("Falling back to OpenCV FileStorage")
            
        # If not new format or parsing failed, try old format with OpenCV FileStorage
        fs = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ)
        
        # Try to read total_configs
        total_configs_node = fs.getNode('total_configs')
        if not total_configs_node.empty():
            total_configs = int(total_configs_node.real())
        else:
            # Count configs manually
            total_configs = 0
            for i in range(1, 20):  # Check up to 20 configurations
                if not fs.getNode(f'config_{i}_position').empty():
                    total_configs = max(total_configs, i)
        
        configs = []
        for i in range(1, total_configs + 1):
            pos_node = fs.getNode(f'config_{i}_position')
            quat_node = fs.getNode(f'config_{i}_quaternion')
            img_node = fs.getNode(f'config_{i}_image')
            
            if not pos_node.empty():
                position = pos_node.mat()
                quaternion = quat_node.mat() if not quat_node.empty() else np.array([1.0, 0.0, 0.0, 0.0])
                image_file = img_node.string() if not img_node.empty() else ""
                
                configs.append({
                    'number': i,
                    'position': position,
                    'quaternion': quaternion,
                    'image_file': image_file
                })
        
        fs.release()
        print(f"Loaded {len(configs)} configurations from old format YAML")
        return configs
        
    except Exception as e:
        print(f"Error loading configurations from YAML: {str(e)}")
        import traceback
        traceback.print_exc()
        return []